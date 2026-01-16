from ultralytics import YOLO

# import torch 

# print(f"torch.cuda.is_available(): {torch.cuda.is_available()}")
# if torch.cuda.is_available():
#     print(f"torch.cuda.device_count(): {torch.cuda.device_count()}")
#     print(f"torch.cuda.get_device_name(0): {torch.cuda.get_device_name(0)}")
# else:
#     print("CUDA is not available.")
# print(f"os.environ['CUDA_VISIBLE_DEVICES']: {os.environ.get('CUDA_VISIBLE_DEVICES')}")


model =YOLO("best.pt")

model.predict('erc_trash_vid.mp4', show=True)