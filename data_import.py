import json
import matplotlib.pyplot as plt

with open('data.json', 'r') as file:
    data = json.load(file)

result = {}
for entry in data:
    step_rate = entry.pop("step_rate")
    pos = list(map(int, entry.keys()))
    spc_values = list(entry.values())
    result[step_rate] = {
        'pos': pos,
        'spc': spc_values
    }

plt.figure(figsize=(10, 6))

for step_rate, values in result.items():
    plt.scatter(values['pos'], values['spc'], label=f'Step Rate: {step_rate}', s=10)
    plt.plot(values['pos'], values['spc'], linestyle='-', alpha=0.7)

plt.xlabel('Position')
plt.ylabel('SPC')
plt.title('SPC vs Position for Different Step Rates')
plt.legend()
plt.grid(True)

plt.savefig('Fig1_SPC.png')
plt.show()
