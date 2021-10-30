import pandas as pd

#this is the file path to the text file
input_file_path = r'/Users/Jonathan Lee/MyFiles/FlightRecorder2/TEST.TXT' 

#this reads in the data
df_flight_data = pd.read_csv(input_file_path, sep=" ", header=None)

#this adds the columns as a header
df_flight_data.columns = ['time', 'heading', 'pitch', 'roll', 'temp', 'pressure', 'altitude']

#this is the output file path ie on your computer
output_file_path = r'/Users/Jonathan Lee/MyFiles/FlightRecorder2/flight_data.csv'

#this writes the data back out to a csv
df_flight_data.to_csv(output_file_path, index=False)
