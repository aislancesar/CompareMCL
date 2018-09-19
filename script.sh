cd Program/AI/Localization/src/
python mindclear.py
cd ../../../Simulator/
python main_simulator.py -r &
cd ../AI/Localization/src/
python Localization.py -g &
python mindcontroller.py -n 50 -s 6 -q 400

# kld = 435
# sd = 470