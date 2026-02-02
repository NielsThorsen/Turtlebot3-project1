import os

# Define the file path (using absolute paths is safer for boot scripts)
file_path = "/home/pi/boot_success.txt"

# Create the empty file
try:
    with open(file_path, "w") as f:
        f.write("Boot script ran successfully!")
    print(f"File created at {file_path}")
except Exception as e:
    print(f"Error: {e}")
