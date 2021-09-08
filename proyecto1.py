import serial
import time
from Adafruit_IO import Client, RequestError, Feed

ADAFRUIT_IO_KEY = "aio_kVZp89bty43jlxjzCrbqjKduYpGD"
ADAFRUIT_IO_USERNAME = "Cue77ar"
aio = Client(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)

luz_feed = aio.feeds('luz')
espacio_feed = aio.feeds('espacio')
sprite_feed = aio.feeds('sprite')
cocacola_feed = aio.feeds('cocacola')

pic = serial.Serial("COM2", 9600)

pic.timeout = 1

time.sleep(1)

print('port conected'+'\n')

while (True) :

    with pic:

        print('Enviando cantidad de cocacolas')

        pic.write(b'a')

        ser_bytes = pic.readline()
        print(ser_bytes)
        cocacolas_bytes= int(ser_bytes)

        print('\n')

        aio.send_data(cocacola_feed.key, cocacolas_bytes)

        print('Enviando cantidad de sprites')

        pic.write(b'b')

        ser_bytes = pic.readline()
        print(ser_bytes)
        sprites_bytes= int(ser_bytes)

        print('\n')

        aio.send_data(sprite_feed.key, sprites_bytes)


        print('Enviando cantidad de espacios')

        pic.write(b'c')

        ser_bytes = pic.readline()
        print(ser_bytes)
        espacios_bytes= int(ser_bytes)

        print('\n')

        aio.send_data(espacio_feed.key, espacios_bytes)


        print('Enviando intensidad de luz')

        pic.write(b'd')

        ser_bytes = pic.readline()
        print(ser_bytes)
        intensidad_bytes= int(ser_bytes[0:len(ser_bytes)-1].decode("utf-8"))

        print('\n')

        aio.send_data(luz_feed.key, intensidad_bytes)


