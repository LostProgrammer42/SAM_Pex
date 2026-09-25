g++ -std=c++17 -g Pex_Test.cpp -o Pex_Test
rm -f *.sampex
./Pex_Test ./INVX_routed.rect invx.sampex
python3 Visualizer.py invx.sampex --animate