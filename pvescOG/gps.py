from pyembedded.gps_module.gps import GPS
import time

gps = GPS(port='/dev/ttyUSB0', baud_rate=9600)

while True:
    data = gps.get_lat_long()
    decoded_data = None

    for encoding in ['utf-8', 'latin-1', 'ascii']:
        try:
            decoded_data = data.decode(encoding)
            break
        except UnicodeDecodeError:
            continue

    if decoded_data is not None:
        print(decoded_data)
    else:
        print("Failed to decode GPS data")

    time.sleep(1)

