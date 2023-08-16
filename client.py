
# This is client code to receive video frames over UDP
import cv2, imutils, socket
import numpy as np
import time
import base64
import torch

# Load a MiDas model for depth estimation
# model_type = "DPT_Large"     # MiDaS v3 - Large     (highest accuracy, slowest inference speed)
#model_type = "DPT_Hybrid"   # MiDaS v3 - Hybrid    (medium accuracy, medium inference speed)
model_type = "MiDaS_small"  # MiDaS v2.1 - Small   (lowest accuracy, highest inference speed)

midas = torch.hub.load("intel-isl/MiDaS", model_type)

# Move model to GPU if available
device = torch.device("cpu")
midas.to(device)
midas.eval()

# Load transforms to resize and normalize the image
midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")

if model_type == "DPT_Large" or model_type == "DPT_Hybrid":
    transform = midas_transforms.dpt_transform
else:
    transform = midas_transforms.small_transform

BUFF_SIZE = 65536
client_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
client_socket.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,BUFF_SIZE)
host_name = socket.gethostname()
host_ip = '192.168.1.112' # socket.gethostbyname(host_name)
print(host_ip)
port = 9999
client_socket.connect((host_ip,port))
message = b'G'

client_socket.sendto(message,(host_ip,port))
fps,st,frames_to_count,cnt = (0,0,20,0)
while True:
	packet,_ = client_socket.recvfrom(BUFF_SIZE)

	data = base64.b64decode(packet,' /')
	npdata = np.fromstring(data,dtype=np.uint8)
	frame = cv2.imdecode(npdata,1)
	frame = cv2.putText(frame,'FPS: '+str(fps),(10,40),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
	cv2.imshow("RECEIVING VIDEO",frame)


	img=frame
	start = time.time()
	img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # Apply input transforms
	input_batch = transform(img).to(device)

    # Prediction and resize to original resolution
    
	with torch.no_grad():
		prediction = midas(input_batch)
		prediction = torch.nn.functional.interpolate(
            prediction.unsqueeze(1),
            size=img.shape[:2],
            mode="bicubic",
            align_corners=False,
        ).squeeze()
		
	depth_map = prediction.cpu().numpy()
	depth_map = cv2.normalize(depth_map, None, 0, 1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_64F)
	end = time.time()
	totalTime = end - start
	
	fps = 1 / totalTime
	img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
	depth_map = (depth_map*255).astype(np.uint8)
	depth_map = cv2.applyColorMap(depth_map , cv2.COLORMAP_MAGMA)
	bw=cv2.cvtColor(depth_map,cv2.COLOR_BGR2GRAY)
	h,w= bw.shape
	s1=bw[0:h , 0:round(w/3)]
	s1=np.sum(s1>200)
	s2=bw[0:h, round(w/3):round(2*w/3)]
	s2=np.sum(s2>200)
	s3=bw[0:h,round(2*w/3):round(w)]
	s3=np.sum(s3>200)
	print("s1=",s1,"  s2=",s2,"s3=   ",s3)
	print(s2)
	n_1 = np.sum(bw>200)
	if(s2>6000):
		print("obstacle alert! : ",end="")
		message = b'N'
		print(s2)
		if(s1>3000):
			print("move right")
			message = b'move right'
		if(s3>3000):
			print("move left")
			message = b'move left'
		else:
			print("Stop")
			message = b'Stop'
	else:
		print("good to go")
		message = b'G'
	
	cv2.putText(img, f'FPS: {int(fps)}', (20,70), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0), 2)
	cv2.imshow('Image', img)
	cv2.imshow('Depth Map', depth_map)
	client_socket.sendto(message,(host_ip,port))


	key = cv2.waitKey(1) & 0xFF
	if key == ord('q'):
		client_socket.close()
		break
	if cnt == frames_to_count:
		try:
			fps = round(frames_to_count/(time.time()-st))
			st=time.time()
			cnt=0
		except:
			pass
	cnt+=1

