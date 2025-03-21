import pandas as pd
import glob

folder_path = "E:\\ppt's\\Masters\\p\\data\\test_data52"

battery_path = folder_path + "\\battery_status.xlsx"
bf = pd.read_excel(battery_path)

# Ensure the 'Timestamp' column is in datetime format
bf['Timestamp'] = pd.to_datetime(bf['Timestamp'])

# Set 'Timestamp' as the index for easy resampling
bf.set_index('Timestamp', inplace=True)

# Resample the data to fill in every second
bf_resampled = bf.resample('1s').ffill().reset_index()
bf_resampled['Timestamp'] = bf_resampled['Timestamp'].dt.strftime('%Y-%m-%d %H:%M:%S')

# Optionally, save the resampled data to a new Excel file
output_path = folder_path + "\\battery_status.xlsx"
bf_resampled.to_excel(output_path, index=False)




# Step 1: Specify the path where the Excel files are stored
allfile_path = folder_path + "/*.xlsx"  # Replace with your actual directory

# Step 2: Create a list of all Excel files in the directory
files = glob.glob(allfile_path)

ignore_list = ['ir_opcode.xlsx',"kidnap_status"]
files_to_process = [f for f in files if not any(ignored in f for ignored in ignore_list)]


# Step 3: Initialize an empty DataFrame for combining all sheets
combined_df = pd.DataFrame()

# Step 4: Loop through each file
for file in files_to_process:
    # Read each sheet in the Excel file
    excel_data = pd.ExcelFile(file)

    # Loop through each sheet in the file
    for sheet_name in excel_data.sheet_names:
        # Read the sheet into a DataFrame
        df = pd.read_excel(file, sheet_name=sheet_name)

        # Append or merge the data based on the timestamp column
        if combined_df.empty:
            combined_df = df
        else:
            combined_df = pd.merge(combined_df, df, on=combined_df.columns[0], how='outer')

# Step 5: Sort the combined DataFrame by the timestamp column
combined_df.sort_values(by=combined_df.columns[0], inplace=True)
combined_df.dropna(inplace=True)

# Step 6: Save the combined DataFrame to a new Excel file
combined_df.to_excel(folder_path+ "\\combined_file.xlsx", index=False)

print("Sheets have been successfully combined and saved as 'combined_file.xlsx'.")
