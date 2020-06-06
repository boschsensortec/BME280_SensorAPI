raspbian: 
	gcc linux_userspace.c ../bme280.c -I ../ -lwiringPi -o bme280

bsd:
	cc bsd_userspace.c ../bme280.c -I ../ -o bme280

clean:
	rm -v bme280
