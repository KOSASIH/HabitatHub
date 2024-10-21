class Communication:
    def __init__(self):
        self.connection_status = False
    
    def establish_connection(self):
        self.connection_status = True
        print("Koneksi ke Bumi berhasil!")
    
    def terminate_connection(self):
        self.connection_status = False
        print("Koneksi ke Bumi dihentikan.")
    
    def send_message(self, message):
        if self.connection_status:
            print(f"Mengirim pesan: {message}")
        else:
            print("Koneksi belum terhubung!")
