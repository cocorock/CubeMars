"""
This script processes CAN data from a CSV file, cleans it, and saves the cleaned data to a new CSV file.
The main steps are:
1. Read the input CSV file.
2. Remove the "name" column and any rows with no data.
3. Merge rows into CAN frames based on the "identifier" column.
4. Remove extra leading zeros from hexadecimal values in the "identifier" and "num_data_bytes" columns.
5. Save the cleaned data to a new CSV file with "_formated" appended to the original filename.

Functions:
- remove_leading_zeros(hexval): Removes extra leading zeros from a hexadecimal string.
- clean_can_data(input_filename): Cleans the CAN data from the input CSV file and saves it to a new CSV file.

Example usage:
input_file = "C.csv"  # Change the file name as needed.
cleaned_data = clean_can_data(input_file)
"""
import pandas as pd
import os

def remove_leading_zeros(hexval):
    """
    Remove extra leading zeros from a hexadecimal string.
    Preserves the "0x" prefix. If nothing remains after stripping,
    returns "0x0".
    """
    if isinstance(hexval, str) and hexval.startswith("0x"):
        trimmed = hexval[2:].lstrip('0')
        if not trimmed:
            trimmed = "0"
        return "0x" + trimmed.upper()  # Optional: uppercase the hex digits
    return hexval

def clean_can_data(input_filename):
    # Change the working directory to the directory where this script is located.
    # This ensures the CSV file is opened from the current directory.
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)

    # Read the CSV file and remove the "name" column.
    df = pd.read_csv(input_filename)
    df = df.drop(columns=["name"])

    # Remove rows that have no data in any column.
    df = df.dropna(how="all")

    # Prepare an empty list to collect each merged CAN frame as a dictionary.
    frames = []

    # This dictionary holds the current CAN frame data while iterating.
    current_frame = {"identifier": None, "num_data_bytes": None, "data": "", "remote_frame": None}

    # Iterate over the rows in the dataframe.
    for idx, row in df.iterrows():
        # If the "identifier" column has data, start a new CAN frame.
        if pd.notna(row["identifier"]):
            # Save the current frame if it exists.
            if current_frame["identifier"] is not None:
                frames.append(current_frame)
                current_frame = {"identifier": None, "num_data_bytes": None, "data": "", "remote_frame": None}

            # Set the identifier and remote_frame from the current row.
            current_frame["identifier"] = row["identifier"]
            current_frame["remote_frame"] = row["remote_frame"]

        # Update num_data_bytes if available.
        if pd.notna(row["num_data_bytes"]):
            current_frame["num_data_bytes"] = row["num_data_bytes"]

        # Append any data row that starts with "0x".
        if pd.notna(row["data"]) and str(row["data"]).startswith("0x"):
            if current_frame["data"]:
                current_frame["data"] += " " + str(row["data"])
            else:
                current_frame["data"] = str(row["data"])

    # Save the last frame if it exists.
    if current_frame["identifier"] is not None:
        frames.append(current_frame)

    # Create a new dataframe from the list of merged CAN frames.
    clean_df = pd.DataFrame(frames)

    # Remove extra leading zeros from "identifier" and "num_data_bytes".
    if "identifier" in clean_df.columns:
        clean_df["identifier"] = clean_df["identifier"].apply(remove_leading_zeros)
    if "num_data_bytes" in clean_df.columns:
        clean_df["num_data_bytes"] = clean_df["num_data_bytes"].apply(remove_leading_zeros)

    # Determine the output filename using the original filename.
    base, ext = os.path.splitext(input_filename)
    output_filename = f"{base}_formated{ext}"

    # Save the cleaned DataFrame to a new CSV file.
    clean_df.to_csv(output_filename, index=False)
    print(f"Cleaned data saved to {output_filename}")
    return clean_df

# Example usage:
input_file = "MIT mode.csv"  # Change the file name as needed.
cleaned_data = clean_can_data(input_file)