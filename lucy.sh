cd helpers
./RandomPopulationGenerator.py
cd ..
sudo nice -n-20 stdbuf -oL ./LucyEvolution.py > out.txt
