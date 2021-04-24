import re
import sys
import os

def parse_one_file(in_file, out_file, tmp_cleanup):

    lines_to_write = []

    with open(in_file, 'r') as f:
        
        for line in f:
            
            clear_line = re.search('clear;', line)
            hold_line = re.search('hold on;', line)

            if clear_line is not None or hold_line is not None:

                continue;

            if tmp_cleanup == 0:

                lines_to_write.append(line)
            else:
                
                color_line = re.search('color', line)

                if color_line is not None:
                    
                    items = line.split('color')

                    new_line = items[0] + 'color\', \'[0 0 1]\');' 
                    
                    lines_to_write.append(new_line)

    with open(out_file, 'w') as stream:

        for line in lines_to_write:

            stream.write(line)    

def main(argv):

    in_dir = argv[0]
    out_dir = argv[1]

    tmp_cleanup = 0

    if not os.path.exists(out_dir):
        os.makedirs(out_dir)

    for name in os.listdir(in_dir):

        in_file = os.path.join(in_dir, name)
        out_file = os.path.join(out_dir, name)

        if os.path.isfile(in_file):

            # check if matlab file
            if '.m' in in_file:

                if 'tmp' in in_file:
                    tmp_cleanup = 0
                else:
                    tmp_cleanup = 1

                parse_one_file(in_file, out_file, tmp_cleanup)
    



if __name__ == '__main__':
    main(sys.argv[1:])
