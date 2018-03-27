#!/usr/bin/python

import argparse
import struct
import collections
import csv

if __name__ =='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('csv', type=str, help='The CSV file to blobify.')

    args = parser.parse_args()

    columns = collections.defaultdict(list)
    print 'Loading CSV'

    with open(args.csv, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            for (k,v) in row.items():
                columns[k].append(int(v))
    print 'Done'

    if len(columns['Sample number']) < 10000000:
        print 'The log does not contain atleast 2 seconds worth of data.'
        sys.exit(-1)

    with open('samples_blob', 'w') as f:
        for i in range(0, 10000000):
            f.write(struct.pack('<hhhh', columns[' C1'][i], columns[' C2'][i], columns[' C3'][i], columns[' C4'][i]))

    print 'Blob written.'
