#!/usr/bin/env python3
import os
import argparse

def filter_and_scale_txt(image_folder, txt_file, output_file, scale):
    """
    Reads the txt_file line by line, and for each line:
      - Checks whether the image (first token) exists in image_folder.
      - If it exists, scales all numeric coordinate values (tokens after the first)
        by the given scale factor.
      - Writes the resulting line to output_file.
    """
    # Get the set of filenames currently in the image folder.
    image_files = set(os.listdir(image_folder))
    
    # Read all lines from the input text file.
    with open(txt_file, 'r') as f:
        lines = f.readlines()
    
    filtered_lines = []
    for line in lines:
        line = line.strip()
        if not line:
            continue  # Skip empty lines
        tokens = line.split()
        # The first token is assumed to be the image filename.
        image_name = tokens[0]
        if image_name in image_files:
            # Process the rest of the tokens by scaling them if they are numeric.
            new_tokens = [image_name]
            for token in tokens[1:]:
                try:
                    value = float(token)
                    scaled_value = value * scale
                    new_tokens.append(str(scaled_value))
                except ValueError:
                    # If conversion to float fails, leave the token unchanged.
                    new_tokens.append(token)
            filtered_lines.append(" ".join(new_tokens))
    
    # Write the filtered (and scaled) lines to the output file.
    with open(output_file, 'w') as f:
        for line in filtered_lines:
            f.write(line + "\n")
    
    print(f"Processed text file written to: {output_file}")

def main():
    parser = argparse.ArgumentParser(
        description=("Filter a text file to remove lines whose image files have been deleted "
                     "from a folder and scale coordinate values by a given factor.")
    )
    # Define four positional arguments.
    parser.add_argument(
        "image_folder",
        type=str,
        help="Path to the folder containing images."
    )
    parser.add_argument(
        "output_folder",
        type=str,
        help="Path to the folder where the filtered text file will be saved."
    )
    parser.add_argument(
        "txt_file",
        type=str,
        help="Path to the input text file."
    )
    parser.add_argument(
        "scale",
        type=float,
        help=("Scale factor to multiply coordinate values by. For example, use 0.01 "
              "to scale values by 1/100.")
    )
    args = parser.parse_args()
    
    # Create output file path by combining output_folder and a fixed file name.
    output_file = os.path.join(args.output_folder, "filtered.txt")
    
    filter_and_scale_txt(args.image_folder, args.txt_file, output_file, args.scale)

if __name__ == "__main__":
    main()
