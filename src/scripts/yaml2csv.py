import yaml
import csv
import argparse
import os
import tf

parser = argparse.ArgumentParser

def to_csv(filepath, format):
    with open(filepath, 'r') as fr:
        yml = yaml.load(fr)
        #print(yml)

        root, ext = os.path.splitext(filepath)


        with open(root + ".csv", 'w') as fw:
            writer = csv.writer(fw, lineterminator='\n')

            # first column
            writer.writerow(['x', 'y', 'z', 'yaw', 'velocity', 'change_flag'])

            velocity = yml['velocity']
        
            for wp in yml['waypoints']:
                euler = tf.transformations.euler_from_quaternion((wp['ox'], wp['oy'], wp['oz'], wp['ow']))
                writer.writerow([wp['px'], wp['py'], wp['pz'], euler[2], velocity, 0])



def create_parser():
    ps = argparse.ArgumentParser(prog="yaml2csv")
    ps.add_argument('-i', '--input', nargs='*', required=True, help='yaml filepath')
    ps.add_argument('-f', '--format', default='2', help='csv format version')
    return ps

def main():
    parser = create_parser()
    args = parser.parse_args()

    for b in args.input:
        to_csv(filepath=b, format=args.format)


if __name__ == '__main__':
    main()