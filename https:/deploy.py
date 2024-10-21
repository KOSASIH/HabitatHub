from habitat.life_support import LifeSupport
from habitat.power_generation import PowerGeneration
from habitat.communication import Communication
from habitat.agent import HabitatAgent

def deploy_habitat():
    # Membuat objek sistem habitat
    life_support = LifeSupport()
    power_gen = PowerGeneration()
    comms = Communication()
    
    # Menghubungkan agen otonom
    agent = HabitatAgent(life_support, power_gen, comms)
    
    # Menjalankan cek harian
    agent.perform_daily_checks()
    
    # Mendirikan komunikasi
    comms.establish_connection()
    comms.send_message("Habitat dalam kondisi baik, semua sistem bekerja dengan baik.")
    
    # Simulasi shutdown darurat
    agent.emergency_shutdown()

if __name__ == "__main__":
    deploy_habitat()
