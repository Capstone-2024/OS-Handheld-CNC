from main import main
from svg import svg_to_points

px_to_mm = 300/100


circle = svg_to_points('./svg_files/circle.svg', px_to_mm)
print(circle)
square = svg_to_points('./svg_files/square.svg', px_to_mm)

def app():

    choice = input(f'Choose a shape to draw. \nCircle: c | Square: s\n')
    if input(f'Begin Operation? Y/N \n') == "Y": 
        if choice == 'c': 
            main(circle)
        elif choice == 's': 
            main(square)
        else: 
            print("Invalid choice, exiting...\n")


if __name__ == "__main__":
    app()