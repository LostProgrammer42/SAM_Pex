g++ -std=c++17 -g Pex_Test.cpp -o Pex_Test
rm -f *.sampex
./pex_test ./INVX_routed.rect invx.sampex
python3 Visualizer.py invx.sampex --animate