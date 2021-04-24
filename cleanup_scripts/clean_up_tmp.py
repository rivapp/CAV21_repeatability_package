import re
import sys

def main(argv):

    input_filename = argv[0]
    output_filename = argv[1]

    lines_to_write = []

    with open(input_filename, 'r') as f:
        
        for line in f:
            
            clear_line = re.search('clear;', line)

            if clear_line is not None:

                continue;

            lines_to_write.append(line)

    with open(output_filename, 'w') as stream:

        for line in lines_to_write:

            stream.write(line)

if __name__ == '__main__':
    main(sys.argv[1:])
