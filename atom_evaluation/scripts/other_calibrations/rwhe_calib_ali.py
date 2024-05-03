#!/usr/bin/env python3

import argparse
import json

def main():
    
    # Parse command line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-json", "--json_file", type=str, required=True,
                    help="Json file containing input dataset.")
    args = vars(ap.parse_args())

    json_file = args['json_file']

    # Read dataset file
    f = open(json_file, 'r')
    dataset = json.load(f)

    # Close dataset file
    f.close()

    print(dataset)  


if __name__ == '__main__':
    main()