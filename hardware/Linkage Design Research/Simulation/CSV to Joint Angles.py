import csv
import matplotlib.pyplot as plt
import math

l1 = 35.9210
l4 = l1  
l2 = 71.84
l3 = l2
l5 = 101.6


def plot_coordinates_from_csv(file_path):
    latitudes = []
    longitudes = []
    jointAngles1 = []
    jointAngles2 = []

    with open(file_path, 'r') as csvfile:
        reader = csv.reader(csvfile)
        for i, row in enumerate(reader):
            if len(row) >= 2:
                
                x = latitude = float(row[0])/10
                y = longitude = float(row[1])/10

                if i == 0:
                    reference_latitude = latitude
                    reference_longitude = longitude

                latitudes.append(latitude - reference_latitude)
                longitudes.append(longitude - reference_longitude)
 
    plt.scatter(latitudes, longitudes)
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('Coordinates Plot')
    plt.show()

    for i in range(len(latitudes)-1):
                
                x = latitudes[i+1]
                y = longitudes[i+1]

                c = math.sqrt(math.pow(x, 2) + math.pow(y, 2))
                e = math.sqrt(math.pow((l5-x), 2) + math.pow(y, 2))
                alpha1 = math.tan(y/x)
                alpha2 = math.degrees(math.acos((math.pow(l2, 2)-math.pow(c, 2)-math.pow(l1, 2)) / (-2*l1*c)))
                alpha = alpha1 + alpha2
                beta1 = math.degrees(math.tan(y/(l5-x)))
                beta2 = math.degrees(math.acos((math.pow(l2, 2) - math.pow(l1, 2) - math.pow(e, 2)) / (-2*l1*e)))
                beta = 180 - (beta1 + beta2)    

                jointAngles1.append(alpha)
                jointAngles2.append(beta)

                print(jointAngles1[i])

# Example usage
csv_file_path = 'circle coordinates.csv'
plot_coordinates_from_csv(csv_file_path)
