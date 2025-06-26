import cv2
import time
 
# Reemplaza la URL por la dirección RTSP de tu cámara FLIR
#rtsp_url = "rtsp://10.2.105.152/avc/?cbr=0&quant=1&frate=29&gop=0"
#rtsp_url = "rtsp://10.2.105.152/mjpg/"
rtsp_url = "rtsp://admin:cv+sg-34a1@10.2.105.189:554/profile2/media.smp"
 
# Crear un objeto VideoCapture con la URL RTSP
cap = cv2.VideoCapture(rtsp_url)
 
# Inicializa las variables para calcular FPS
fps = 0
frame_count = 0
start_time = time.time()
 
while True:
    ret, frame = cap.read()
    # print(ret)

    if ret:
        frame_count += 1
        end_time = time.time()
        time_diff = end_time - start_time
 
        # Calcula FPS cada segundo para mejorar la estabilidad de la lectura
        if time_diff >= 1:
            fps = frame_count / time_diff
            start_time = time.time()
            frame_count = 0
 
        # Mostrar FPS en el video
        cv2.putText(frame, f'FPS: {fps:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
 
        cv2.imshow("FLIR Camera Stream", frame)
 
        # Presiona 'q' para salir
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        print("No se pudo capturar el video.")
        break
 
cap.release()
cv2.destroyAllWindows()
 
