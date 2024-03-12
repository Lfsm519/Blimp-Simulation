# Running The Simulation

Owner: drMoscovium
Tags: Research

Start by Visiting the repository `Blimp-Simulation` 

![Screenshot 2024-02-16 at 3.36.41 PM.png](Running%20The%20Simulation%2031aafa45d3814f6494cf1adfbc4ad400/Screenshot_2024-02-16_at_3.36.41_PM.png)

Clone the repository to your working folder

```bash
git clone git@github.com:Blimp-Senior-Design/Blimp-Simulation.git
```

Checkout my branch

```bash
cd Blimp-Simulation
git checkout dev/yojan
python3 -m venv env
source env/bin/activate
python3 -m pip install -r requirements.txt
mjpython main.py
```

To update your branch you run the following command

```bash
git pull
```