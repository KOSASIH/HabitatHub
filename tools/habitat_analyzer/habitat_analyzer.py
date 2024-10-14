# habitat_analyzer.py

import numpy as np

class HabitatAnalyzer:
    def __init__(self, habitat_simulation):
        self.habitat_simulation = habitat_simulation
        self.analysis = {}

    def analyze_habitat(self):
        # Analyze the habitat based on the simulation
        self.analysis['environment'] = self.analyze_environment()
        self.analysis['systems'] = self.analyze_systems()
        self.analysis['agents'] = self.analyze_agents()

        return self.analysis

    def analyze_environment(self):
        # Analyze the environment of the habitat
        environment = self.habitat_simulation['environment']
        analysis = {
            'temperature': np.mean(environment['temperature']),
            'humidity': np.mean(environment['humidity']),
            'atmosphere': np.mean(environment['atmosphere'])
        }
        return analysis

    def analyze_systems(self):
        # Analyze the systems of the habitat
        systems = self.habitat_simulation['systems']
        analysis = {
            'life_support': np.mean(list(systems['life_support'].values())),
            'power': np.mean(list(systems['power'].values())),
            'communication': np.mean(list(systems['communication'].values()))
        }
        return analysis

    def analyze_agents(self):
        # Analyze the agents in the habitat
        agents = self.habitat_simulation['agents']
        analysis = {
            'agent_count': len(agents),
            'agent_locations': [agent['location'] for agent in agents]
        }
        return analysis

def main():
    habitat_designer = HabitatDesigner('space_station', (100, 100, 100))
    habitat_design = habitat_designer.design_habitat()
    habitat_simulator = HabitatSimulator(habitat_design)
    habitat_simulation = habitat_simulator.simulate_habitat()
    habitat_analyzer = HabitatAnalyzer(habitat_simulation)
    analysis = habitat_analyzer.analyze_habitat()
    print(analysis)

if __name__ == '__main__':
    main()
