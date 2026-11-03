g++ -O2 -std=c++17 Pex_Test.cpp -o pex
./pex INVX1.def tech.cfg out.sampex
python Visualizer.py out.sampex