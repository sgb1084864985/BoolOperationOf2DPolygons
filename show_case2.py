import show_polygon
import argparse
from matplotlib import pyplot as plt
from glob import glob
if __name__=='__main__':
  # Create a parser object with a description of the program
  parser = argparse.ArgumentParser(description="show Polygon from folder test cases.")
  # Add an argument for the input files with type=argparse.FileType('r') and nargs='+'
  parser.add_argument("case_num", type=int, help="case number")
  parser.add_argument("-r", type=str, help="show result",dest="result")

  # Parse the command-line arguments and get the input file objects
  args = parser.parse_args()

  polygon_path=f'./test_cases/case{args.case_num}'

  if args.result is None:
    input_files=[polygon_path+'/a.txt',polygon_path+'/b.txt']
  else:
    input_files=glob(f'{polygon_path}/{args.result}V2*')

  plt.xlim(-1.1, 1.1)
  plt.ylim(-1.1, 1.1)
  # Loop through each input file object
  for f in input_files:
    # Read the points from the input file object
    points = show_polygon.read_points(f)
    # Show the points as a polygon
    show_polygon.show_polygon(points)
    # Show the plot on the screen
  plt.show()