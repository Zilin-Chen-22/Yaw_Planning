import csv

def read_csv_columns():
    col1, col2, col3 = [], [], []

    with open('data.csv', 'r', encoding='utf-8') as csvfile:
        reader = csv.reader(csvfile)

        for row in reader:
            if len(row) >= 3:
                col1.append(float(row[0]))
                col2.append(float(row[1]))
                col3.append(float(row[2]))

    return [col1, col2, col3]
