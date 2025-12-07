import urllib.request
import os

# URLs alternativas (Mirror mais estável para OpenCV)
URL_PROTOTXT = "https://raw.githubusercontent.com/djmv/MobilNet_SSD_opencv/master/MobileNetSSD_deploy.prototxt"
URL_MODEL = "https://github.com/djmv/MobilNet_SSD_opencv/raw/master/MobileNetSSD_deploy.caffemodel"

FILES = {
    "MobileNetSSD_deploy.prototxt": URL_PROTOTXT,
    "MobileNetSSD_deploy.caffemodel": URL_MODEL
}

def download_file(filename, url):
    print(f"[BAIXANDO] {filename}...")
    try:
        # Configurar um User-Agent para evitar erro 403/404 do GitHub (bloqueio de bots)
        opener = urllib.request.build_opener()
        opener.addheaders = [('User-Agent', 'Mozilla/5.0 (Windows NT 10.0; Win64; x64)')]
        urllib.request.install_opener(opener)
        
        urllib.request.urlretrieve(url, filename)
        
        # Verificar se baixou algo > 0 bytes
        size = os.path.getsize(filename)
        if size < 1000: # Se for muito pequeno, provavel que seja erro de texto html
            print(f"[AVISO] Arquivo {filename} parece muito pequeno ({size} bytes). Pode estar corrompido.")
        else:
            print(f"[SUCESSO] {filename} baixado ({size/1024:.2f} KB).")
            
    except Exception as e:
        print(f"[ERRO] Falha ao baixar {filename}: {e}")

def main():
    print("--- SETUP DO PROJETO DE VISÃO COMPUTACIONAL ---")
    print("Baixando arquivos necessários para a mesma pasta...")
    
    for filename, url in FILES.items():
        if os.path.exists(filename):
            print(f"[AVISO] {filename} já existe. Pulando download.")
            # Opcional: remover o arquivo antigo se quiser forçar o download
            # os.remove(filename) 
        else:
            download_file(filename, url)
            
    print("\n--- CONCLUSÃO ---")
    if os.path.exists("MobileNetSSD_deploy.caffemodel") and os.path.exists("MobileNetSSD_deploy.prototxt"):
        print("Tudo pronto! Agora você pode rodar o 'visao_passaros.py'.")
    else:
        print("Algo deu errado. Verifique sua conexão com a internet.")

if __name__ == "__main__":
    main()