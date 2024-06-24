
main: angle2d.o vector2d.o rotation3d.o vector3d.o main.o
	g++ -o main angle2d.o vector2d.o rotation3d.o vector3d.o main.o

angle2d.o:
	g++ -o angle2d.o -c angle2d.cc
vector2d.o:
	g++ -o vector2d.o -c vector2d.cc
rotation3d.o:
	g++ -o rotation3d.o -c rotation3d.cc
vector3d.o:
	g++ -o vector3d.o -c vector3d.cc
main.o:
	g++ -o main.o -c main.cc


clean:
	rm -f angle2d.o vector2d.o rotation3d.o vector3d.o main.o main
