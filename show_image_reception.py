import socket
import struct
import cv2
import numpy as np

HOST = "0.0.0.0"
PORT = 9000

# Placeholder para a função que corrigimos
def recv_exact(conn, size):
    """Receive exactly N bytes from TCP."""
    data = b""
    while len(data) < size:
        try:
            packet = conn.recv(size - len(data))
        except (socket.error, ConnectionResetError, TimeoutError):
            return None # Retorna None em caso de erro
            
        if not packet:
            # Conexão fechada pelo cliente (graceful shutdown)
            return None
            
        data += packet
    return data

# --- Set up TCP server ---
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((HOST, PORT))
# Mantenha o listen fora do loop, pois é uma configuração do servidor.
sock.listen(1) 

print(f"Server is running. Listening on port {PORT}...")

# 🚨 LOOP EXTERNO: Para manter o servidor rodando e aceitar novas conexões
try:
    while True:
        print("\nWaiting for a new connection...")
        conn = None # Zera a conexão antes de aceitar uma nova
        
        # 1. Tenta aceitar uma nova conexão
        try:
            conn, addr = sock.accept()
            print("Connected:", addr)
            
            # 2. LOOP INTERNO: Processamento dos frames do cliente atual
            try:
                while True:
                    # Receive 4-byte size header
                    header = recv_exact(conn, 4)
                    if header is None:
                        print("Saindo da sessão: Conexão encerrada pelo cliente ou erro.")
                        break

                    size = struct.unpack("!I", header)[0]

                    # Receive JPEG image bytes
                    data = recv_exact(conn, size)
                    if data is None:
                        break

                    # Decode JPEG to image e processamento
                    np_data = np.frombuffer(data, dtype=np.uint8)
                    frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)

                    if frame is None:
                        print("Aviso: Falha ao decodificar imagem (dados corrompidos?).")
                        continue

                    # Display last image received
                    cv2.imshow("Last Received Frame", frame)

                    # Close window with 'q'
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        print("Saindo da sessão: Usuário pressionou 'q'.")
                        break
                        
            except Exception as proc_e:
                # Captura erros no processamento (ex: desempacotamento)
                print(f"Erro no processamento de frame: {proc_e}")
            
            # Este 'finally' será executado após o break ou erro de processamento
            finally:
                # Limpa os recursos do cliente APENAS
                if conn:
                    conn.close()
                cv2.destroyAllWindows()
                print("Sessão finalizada. Limpeza dos recursos do cliente realizada.")
                
        except socket.error as sock_e:
            # Erros durante o accept() ou outros erros de socket
            if sock_e.errno == 10004: # Código de erro comum após socket.close()
                print("Servidor desligado.")
                break 
            print(f"Erro de Socket durante 'accept': {sock_e}")
            
        # O loop externo 'while True' recomeça aqui, chamando sock.accept() novamente.
        
except KeyboardInterrupt:
    # Captura Ctrl+C para sair do loop externo
    print("\n[KeyboardInterrupt]: Servidor desligado manualmente.")

finally:
    # 🚨 Limpeza final: Fecha o socket de escuta (servidor)
    sock.close()
    print("Recursos do servidor principal (socket de escuta) fechados.")
    cv2.destroyAllWindows()