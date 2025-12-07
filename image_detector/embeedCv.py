import cv2
import numpy as np
import time

# --- CONFIGURAÇÕES ---
PROTOTXT = "MobileNetSSD_deploy.prototxt"
MODEL = "MobileNetSSD_deploy.caffemodel"
CONFIDENCE_THRESHOLD = 0.5
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
           "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
           "sofa", "train", "tvmonitor"]

COLOR_BIRD = (0, 255, 0)
COLOR_OTHER = (0, 0, 255)

def encontrar_camera_disponivel():
    """Tenta encontrar uma câmera ativa testando índices comuns."""
    # No PC, geralmente é 0 ou 1.
    indices_para_testar = [1, 2, -1]
    
    for idx in indices_para_testar:
        print(f"[DEBUG] Testando webcam no índice {idx}...")
        cap = cv2.VideoCapture(idx)
        # Tenta forçar o backend DirectShow no Windows se o padrão falhar (opcional)
        # cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW) 
        
        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None:
                print(f"[SUCESSO] Webcam encontrada no índice {idx}!")
                return cap
            else:
                print(f"[AVISO] Webcam abriu no índice {idx}, mas retornou imagem preta/vazia.")
                cap.release()
        else:
            print(f"[AVISO] Nenhuma webcam no índice {idx}.")
    
    return None

def main():
    print("[INFO] Carregando modelo MobileNetSSD...")
    try:
        net = cv2.dnn.readNetFromCaffe(PROTOTXT, MODEL)
    except cv2.error:
        print("[ERRO] Arquivos do modelo não encontrados!")
        print(f"Certifique-se de baixar '{PROTOTXT}' e '{MODEL}' e colocar na mesma pasta.")
        return

    print("[INFO] Procurando webcam...")
    cap = encontrar_camera_disponivel()
    
    if cap is None:
        print("\n[ERRO CRÍTICO] Nenhuma webcam foi encontrada!")
        print("VERIFICAÇÕES PARA PC:")
        print("1. Feche outros programas que usam a câmera (Zoom, Teams, Meet).")
        print("2. Verifique as configurações de privacidade do Windows/Mac (Permitir que apps acessem a câmera).")
        print("3. Se usar antivirus, veja se ele não bloqueou o script Python.")
        return

    time.sleep(1.0) # Câmera esquentando
    bird_detected_start_time = None
    ALIMENTANDO = False

    print("[INFO] Sistema rodando no PC. Pressione 'q' na janela para sair.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[ERRO] Falha ao ler frame da câmera.")
            break

        # Redimensiona para o tamanho que a rede neural exige (300x300)
        h, w = frame.shape[:2]
        resized_frame = cv2.resize(frame, (300, 300))
        blob = cv2.dnn.blobFromImage(resized_frame, 0.007843, (300, 300), 127.5)
        
        net.setInput(blob)
        detections = net.forward()

        passaro_presente_neste_frame = False

        # Loop pelas detecções
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            
            if confidence > CONFIDENCE_THRESHOLD:
                idx = int(detections[0, 0, i, 1])
                class_name = CLASSES[idx]
                
                # Coordenadas do quadrado
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")

                if class_name == "bird":
                    passaro_presente_neste_frame = True
                    
                    # Desenha quadrado VERDE
                    label = "{}: {:.2f}%".format(class_name, confidence * 100)
                    cv2.rectangle(frame, (startX, startY), (endX, endY), COLOR_BIRD, 2)
                    y = startY - 15 if startY - 15 > 15 else startY + 15
                    cv2.putText(frame, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_BIRD, 2)
                else:
                    # Desenha quadrado VERMELHO (outros objetos) para debug
                    # Pode comentar essas linhas abaixo se quiser limpar a tela
                    label = "{}: {:.2f}%".format(class_name, confidence * 100)
                    cv2.rectangle(frame, (startX, startY), (endX, endY), COLOR_OTHER, 2)
                    y = startY - 15 if startY - 15 > 15 else startY + 15
                    cv2.putText(frame, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_OTHER, 2)

        # Lógica de validação temporal (anti-falso positivo)
        if passaro_presente_neste_frame:
            if bird_detected_start_time is None:
                bird_detected_start_time = time.time()
            
            tempo_presenca = time.time() - bird_detected_start_time
            
            # Desenha cronômetro na tela para facilitar o debug visual
            cv2.putText(frame, f"Presenca: {tempo_presenca:.1f}s", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            if tempo_presenca > 2.0 and not ALIMENTANDO:
                msg = f"[AÇÃO] ALIMENTAR! (Pássaro confirmado por {tempo_presenca:.1f}s)"
                print(msg)
                # Visualmente mostrar que está alimentando
                cv2.putText(frame, "ALIMENTANDO...", (10, 60), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                ALIMENTANDO = True 
        else:
            bird_detected_start_time = None
            ALIMENTANDO = False

        cv2.imshow("Visao Computacional - PC", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    print("[INFO] Encerrando...")
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()