from pytagmapper_tools.build_map import build_map
from pytagmapper_tools.hack_sys_path import *
from pytagmapper_tools.make_aruco_tag_txts import aruco_tag_data
from pytagmapper.data import *
from pytagmapper.geometry import *
from pytagmapper.project import *
from pytagmapper.inside_out_tracker import InsideOutTracker
from pytagmapper.rolling_mean_var import RollingMeanVar
from threading_utils import WebcamVideoStream
from svg import svg_to_points
import cv2, time, glob, os


def main():
    if len(glob.glob("./data/*.png")) == 0:
        # Image taking sequence
        # Start Camera Stream and FPS Counter
        vs = WebcamVideoStream(src=0).start()

        i = 0
        while True:
            frame = vs.read()

            cv2.imshow("frame", frame)

            if chr(cv2.waitKey(1) & 255) == "c":  # capture key, change to button later
                cv2.imwrite("./data/image_" + str(i) + ".png", frame)

                i += 1
                print("Photo saved... Press 'q' to exit or continue taking photos. \n")

            elif chr(cv2.waitKey(1) & 255) == "q":  # Change to button click later
                break

        cv2.destroyAllWindows()
        vs.stop()

        print("Capture finished. Now building map ... Please wait. \n")

    """ Build Map """
    # Build Aruco Txt Data Files
    data_dir = os.path.abspath("./data")

    if len(glob.glob("./data/*.txt")) <= 1:
        aruco_tag_data(data_dir, True)

    if len(glob.glob("./data/*.json")) == 0:
        build_map(data_dir, data_dir, "2d")
        # Display some stuff about the map - maybe size or whatever
        print("Finished. \n")

    """ Load Desired Points Data """
    # px_to_mm = 300/100

    # circle = svg_to_points('./svg_files/circle.svg', px_to_mm)
    # square = svg_to_points('./svg_files/square.svg', px_to_mm)

    # choice = input(f'Choose a shape to draw. \nCircle: c | Square: s\n')
    # if input(f'Begin Operation? Y/N \n') == "Y":
    #     if choice == 'c':
    #         main(circle)
    #     elif choice == 's':
    #         main(square)
    #     else:
    #         print("Invalid choice, exiting...\n")

    """ ------------------------------- Set up ------------------------------- """
    map_data = load_map(data_dir)
    map_type = map_data["map_type"]
    camera_matrix = load_camera_matrix(data_dir)
    distortion_matrix = load_distortion_matrix(data_dir)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    # BGR format
    aruco_side_colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)]

    axes_colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0)]

    tag_min_x = float("inf")
    tag_min_y = float("inf")
    tag_max_x = -float("inf")
    tag_max_y = -float("inf")

    # Decompose Map data
    for pose_world_tag in map_data["tag_locations"].values():
        x = pose_world_tag[0]
        y = pose_world_tag[1]

        tag_min_x = min(x, tag_min_x)
        tag_min_y = min(y, tag_min_x)
        tag_max_x = max(x, tag_max_x)
        tag_max_y = max(y, tag_max_x)

    map_dimension = max(tag_max_x - tag_min_x, tag_max_y - tag_min_y)

    viz_height = 10
    x_center = (tag_min_x + tag_max_x) / 2
    y_center = (tag_min_y + tag_max_y) / 2
    tdcam_display_width = 800
    tx_world_tdcam = np.array(
        [
            [1, 0, 0, x_center],
            [0, -1, 0, y_center],
            [0, 0, -1, viz_height],
            [0, 0, 0, 1],
        ]
    )

    # world coordinate
    tx_tdcam_world = SE3_inv(tx_world_tdcam)

    f = 2 * tdcam_display_width / map_dimension
    tdcam_matrix = np.array(
        [
            [f, 0, tdcam_display_width / 2],
            [0, f, tdcam_display_width / 2],
            [0, 0, 1],
        ]
    )

    # project camera axes to topdown
    axes = np.array(
        [[0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1], [1, 1, 1, 1]], dtype=np.float64
    )
    axes[:3, :] *= 0.1

    camera = cv2.VideoCapture(0, cv2.CAP_DSHOW)

    tracker_initted = False
    tracker = InsideOutTracker(camera_matrix, map_data)

    mv_x = RollingMeanVar(10)
    mv_y = RollingMeanVar(10)
    mv_z = RollingMeanVar(10)

    """ ------------------------------- Main Loop ------------------------------- """

    while True:
        # add the threading
        ret, frame = camera.read()
        if not ret:
            continue

        aruco_corners, aruco_ids, aruco_rejected = detector.detectMarkers(frame)

        if aruco_corners is None:
            aruco_corners = []
        if aruco_ids is None:
            aruco_ids = []
        else:
            aruco_ids = aruco_ids.flatten()

        # print(aruco_corners)
        # print(aruco_ids)

        aruco_corners_flat = [np.array([c.flatten()]).T for c in aruco_corners]
        for tag_id, acorners in zip(aruco_ids, aruco_corners_flat):
            for i in range(4):
                ni = (i + 1) % 4
                start = (int(acorners[2 * i, 0]), int(acorners[2 * i + 1, 0]))
                end = (int(acorners[2 * ni, 0]), int(acorners[2 * ni + 1, 0]))
                color = aruco_side_colors[i]
                cv2.line(frame, start, end, color, thickness=1)

            center = np.sum(acorners.reshape((4, 2)), axis=0) / 4
            cv2.putText(
                frame,
                str(tag_id),
                (int(center[0]), int(center[1])),
                0,
                1,
                (0, 255, 255),
                thickness=1,
            )

        if len(aruco_ids) and not tracker_initted:
            print("Initting...")
            while tracker.error < 100:
                improved = tracker.update(
                    aruco_ids, aruco_corners_flat, force_update=False
                )
            tracker_initted = True
            print("Done")

        if tracker_initted:
            improved = tracker.update(aruco_ids, aruco_corners_flat, force_update=True)

        ptag_ids, ptag_corners = tracker.get_projections()
        for tag_id, acorners in zip(ptag_ids, ptag_corners):
            for i in range(4):
                ni = (i + 1) % 4
                start = (int(acorners[2 * i, 0]), int(acorners[2 * i + 1, 0]))
                end = (int(acorners[2 * ni, 0]), int(acorners[2 * ni + 1, 0]))
                color = aruco_side_colors[i]
                cv2.line(frame, start, end, color, thickness=1)

            center = np.sum(acorners.reshape((4, 2)), axis=0) / 4
            cv2.putText(
                frame,
                str(tag_id),
                (int(center[0]), int(center[1])),
                0,
                1,
                (0, 0, 255),
                thickness=1,
            )

        scale = 400 / frame.shape[1]
        resize_shape = (int(frame.shape[1] * scale), int(frame.shape[0] * scale))
        resized = cv2.resize(frame, resize_shape)

        topdown_viz = np.zeros(
            (int(tdcam_display_width), int(tdcam_display_width), 3), np.uint8
        )
        topdown_viz[-resize_shape[1] :, -resize_shape[0] :, :] = resized

        # project all tags to topdown
        for tag_id, tx_world_tag in tracker.txs_world_tag.items():
            tx_tdcam_tag = tx_tdcam_world @ tx_world_tag
            tag_side_length = map_data["tag_side_lengths"].get(tag_id, None)
            if tag_side_length is None:
                tag_side_length = map_data["tag_side_lengths"]["default"]
            corners_mat = get_corners_mat(tag_side_length)
            corners_td, _, _ = project(tdcam_matrix, tx_tdcam_tag, corners_mat)
            for i in range(4):
                ni = (i + 1) % 4
                start = (int(corners_td[2 * i, 0]), int(corners_td[2 * i + 1, 0]))
                end = (int(corners_td[2 * ni, 0]), int(corners_td[2 * ni + 1, 0]))
                color = aruco_side_colors[i]
                cv2.line(topdown_viz, start, end, color, thickness=1)

            center = np.sum(corners_td.reshape((4, 2)), axis=0) / 4
            cv2.putText(
                topdown_viz,
                str(tag_id),
                (int(center[0]), int(center[1])),
                0,
                1,
                (0, 0, 255),
                thickness=1,
            )

        axes_img = (
            tdcam_matrix @ (tx_tdcam_world @ tracker.tx_world_viewpoint @ axes)[:3, :]
        )
        axes_img[:2, :] /= axes_img[2, :]
        axes_origin = axes_img[:, 0]

        for i in range(3):
            end = axes_img[:, i + 1]
            cv2.line(
                topdown_viz,
                (int(axes_origin[0]), int(axes_origin[1])),
                (int(end[0]), int(end[1])),
                axes_colors[i],
                thickness=1,
            )

        cv2.putText(
            topdown_viz, f"q = quit", (10, 30), 0, 0.5, (255, 255, 255), thickness=1
        )
        cv2.putText(
            topdown_viz,
            f"Tracker Error: {tracker.error}",
            (10, 60),
            0,
            0.5,
            (255, 255, 255),
            thickness=1,
        )

        rotation_y = 90
        cv2.putText(
            topdown_viz,
            str("Rotation"),
            (10, rotation_y),
            0,
            0.5,
            (255, 255, 255),
            thickness=1,
        )
        cv2.putText(
            topdown_viz,
            str(tracker.tx_world_viewpoint[0, :3]),
            (10, rotation_y + 15),
            0,
            0.5,
            (255, 255, 255),
            thickness=1,
        )
        cv2.putText(
            topdown_viz,
            str(tracker.tx_world_viewpoint[1, :3]),
            (10, rotation_y + 2 * 15),
            0,
            0.5,
            (255, 255, 255),
            thickness=1,
        )
        cv2.putText(
            topdown_viz,
            str(tracker.tx_world_viewpoint[2, :3]),
            (10, rotation_y + 3 * 15),
            0,
            0.5,
            (255, 255, 255),
            thickness=1,
        )

        xyz_y = rotation_y + 5 * 15
        cv2.putText(
            topdown_viz, str("XYZ"), (10, xyz_y), 0, 0.5, (255, 255, 255), thickness=1
        )
        cv2.putText(
            topdown_viz,
            str(tracker.tx_world_viewpoint[:3, 3]),
            (10, xyz_y + 15),
            0,
            0.5,
            (255, 255, 255),
            thickness=1,
        )

        mv_x.add_datum(tracker.tx_world_viewpoint[0, 3])
        mv_y.add_datum(tracker.tx_world_viewpoint[1, 3])
        mv_z.add_datum(tracker.tx_world_viewpoint[2, 3])

        # cv2.putText(topdown_viz, f"X mean std {mv_x.mean:#.4g} {mv_x.var**0.5:#.4g}", (10, xyz_y + 30), 0, 0.5, (255, 255, 255), thickness=1)
        # cv2.putText(topdown_viz, f"Y mean std {mv_y.mean:#.4g} {mv_y.var**0.5:#.4g}", (10, xyz_y + 45), 0, 0.5, (255, 255, 255), thickness=1)
        # cv2.putText(topdown_viz, f"Z mean std {mv_z.mean:#.4g} {mv_z.var**0.5:#.4g}", (10, xyz_y + 60), 0, 0.5, (255, 255, 255), thickness=1)

        cv2.imshow("Inside Out Tracking Demo", topdown_viz)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    camera.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()