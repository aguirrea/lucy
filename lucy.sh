cd helpers
./RandomPopulationGenerator.py
cd ..
stdbuf -oL ./LucyEvolution.py > out.txt
