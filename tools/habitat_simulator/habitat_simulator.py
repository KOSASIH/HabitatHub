# habitat_simulator.py

import numpy as np

class HabitatSimulator:
    def __init__(self, habitat_design):
        self.habitat_design = habitat_design
        self.simulation = {}

    def simulate_habitat(self):
        # Simulate the habitat based on the design
        self.simulation['environment'] = self.simulate_environment()
        self.simulation['systems'] = self.simulate_systems()
        self.simulation['agents'] = self.simulate_agents()

        return self.simulation

    def simulate_environment(self):
        # Simulate the environment of the habitat
        environment = {
            'temperature': np.random.uniform(20, 30),
            'humidity': np.random.uniform(50, 70),
            'atmosphere': np.random.uniform(100, 110)
        }
        return environment

    def simulate_systems(self):
        # Simulate the systems of the habitat
        systems = {
            'life_support': {'oxygen': 90, 'water': 80, 'food': 70},
            'power': {'solar_panels': 8, 'batteries': 9},
            'communication': {'antennas': 4, 'transceivers': 5}
        }
        return systems

    def simulate_agents(self):
        # Simulate the agents in the habitat
        agents = []
        for i in range(5):
            agent = {
                'name': f'Agent {i+1}',
                'type': 'astronaut',
                'location': (np.random.uniform(0, 100), np.random.uniform(0, 100), np.random.uniform(0, 100))
            }
            agents.append(agent)
        return agents

def main():
    habitat_designer = HabitatDesigner('space_station', (100, 100, 100))
    habitat_design = habitat_designer.design_habitat()
    habitat_simulator = HabitatSimulator(habitat_design)
    simulation = habitat_simulator.simulate_habitat()
    print(simulation)

if __name__ == '__main__':
    main()
