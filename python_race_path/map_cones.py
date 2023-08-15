#!/usr/bin/env python3

import csv
import matplotlib.pyplot as plt

def read_traffic_cones_csv(csv_filename):
    cones = []  # Array to store traffic cones

    with open(csv_filename, 'r') as csvfile:
        csv_reader = csv.reader(csvfile)
        print('csv file read')

        for row in csv_reader:
            if row[0] == 'blue': color = 0
            elif row[0] == 'yellow': color = 1
            else: color = 2
            x_pos = float(row[1])  # Assuming the X position is in the first column
            y_pos = float(row[2])  # Assuming the Y position is in the second column            

            cone = {
                'x': x_pos,
                'y': y_pos,
                'color': color
            }

            cones.append(cone)

    return cones

if __name__ == "__main__":
    
    csv_filename = "track_droneport.csv"  # Change to your CSV file's name
    cones = read_traffic_cones_csv(csv_filename)

    # Plot the traffic cones
    for cone in cones:
        if cone['color'] == 0:
            plt.scatter(cone['x'], cone['y'], color='blue')
        elif cone['color'] == 1:
            plt.scatter(cone['x'], cone['y'], color='yellow')
        elif cone['color'] == 2:
            plt.scatter(cone['x'], cone['y'], color='orange')
            
    # Set plot labels and title
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Traffic Cone Plot')
    plt.axis('equal') 

    # Show the plot
    plt.show()

