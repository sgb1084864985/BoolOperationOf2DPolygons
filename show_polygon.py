# Import the argparse library for command-line parsing
import argparse

# Import the matplotlib library for plotting
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

# Define a function to read a sequence of 2d points from a txt file
def read_points(filename):
  # Create an empty list to store the points
  points_array=[]
  points = []
  # Open the file in read mode
  with open(filename, "r") as f:
    # Loop through each line in the file
    for line in f:
      if line.startswith('#loop'):
        if len(points)>0:
          points_array.append(points)
        points=[]
        continue
      # Split the line by whitespace and convert to float
      x, y = map(float, line.split())
      # Append the point as a tuple to the list
      points.append((x, y))
  # Return the list of points
  if len(points)>0:
    points_array.append(points)
  return points_array

# Define a function to show the points as a polygon
def show_polygon(points_array):
  for points in points_array:
    # Unpack the x and y coordinates from the list of points
    points.append(points[0])
    x, y = zip(*points)
    # Plot the points as a polygon with a blue outline and no fill
    plt.plot(x, y, "b-")
    plt.scatter(x, y, c="r")

if __name__=='__main__':
  # Create a parser object with a description of the program
  parser = argparse.ArgumentParser(description="Plot polygons from txt files of 2d points.")
  # Add an argument for the input files with type=argparse.FileType('r') and nargs='+'
  parser.add_argument("input_files", type=str, nargs='+', help="The names of the txt files containing the 2d points.")
  # Parse the command-line arguments and get the input file objects
  args = parser.parse_args()
  # Loop through each input file object
  for f in args.input_files:
    # Read the points from the input file object
    points = read_points(f)
    # Show the points as a polygon
    show_polygon(points)
    # Show the plot on the screen
  plt.show()
