import json, matplotlib.pyplot as plt

# Use for simulation testing
with open("/home/jarred/git/DalESelfEBot/ur3_control/config/sim_circle_waypoints.json") as f: exp = json.load(f)["splines"][0]["waypoints"]
with open("/home/jarred/git/DalESelfEBot/ur3_control/scripts/sim_ground_truth.json") as f: act = json.load(f)["splines"][0]["waypoints"]

# Use for hardware testing
# with open("/home/jarred/git/DalESelfEBot/ur3_control/config/hardware_circle_waypoints.json") as f: exp = json.load(f)["splines"][0]["waypoints"]
# with open("/home/jarred/git/DalESelfEBot/ur3_control/scripts/hardware_ground_truth.json") as f: act = json.load(f)["splines"][0]["waypoints"]

x_exp, y_exp = zip(*[(p[0], p[1]) for p in exp])
x_act, y_act = zip(*[(p[0], p[1]) for p in act])

plt.plot(x_exp, y_exp, 'r-', label="Expected")
plt.plot(x_act, y_act, 'b--', label="Actual")
plt.gca().set_aspect('equal')
plt.legend(), plt.grid(True), plt.title("Expected vs Actual Path")
plt.show()
