import torch
import torchvision.models as models
import torchvision.transforms as transforms
from PIL import Image
import os
import pandas as pd
from tqdm import tqdm

device = "cuda" if torch.cuda.is_available() else "cpu"
print(f"Using device: {device}")

# Load pretrained CNN (ResNet18 without final classification layer)
resnet18 = models.resnet18(weights=models.ResNet18_Weights.IMAGENET1K_V1)
# resnet18 = resnet18.to(device)
resnet18.eval()  # inference mode
feature_extractor = torch.nn.Sequential(*list(resnet18.children())[:-1])  # remove final FC layer

# print(feature_extractor)

transform = transforms.Compose([
      transforms.Resize((224, 224)),
      transforms.ToTensor(),
      transforms.Normalize(mean=[0.485, 0.456, 0.406],
                              std =[0.229, 0.224, 0.225])
])

def encode_image(image_path):
      img = Image.open(image_path).convert("RGB")
      img_tensor = transform(img).unsqueeze(0)  # add batch dim
      # img_tensor = img_tensor.to(device)
      with torch.no_grad():
            features = feature_extractor(img_tensor).squeeze()  # shape: (512,)
      return features.numpy()

conditioned_vars = pd.read_csv("CVAE/data/conditioned_vars.csv")

encoded_images = []
print(len(conditioned_vars["scenario"].unique()))
img_base_path = "/home/kareem/frenet_optimal_trajectory_planner/CVAE/data/scenarios_imgs"

for scenario in tqdm(conditioned_vars["scenario"].unique(), 
                     total=len(conditioned_vars["scenario"].unique())):
      # scenario = row["scenario"]
      # print(scenario)
      for img in os.listdir(os.path.join(img_base_path, scenario)):
            if img.endswith(".png"):
                  img_path = os.path.join(img_base_path, scenario, f"{img}")
            
                  if os.path.exists(img_path):
                        vec = encode_image(img_path)
                        encoded_images.append(vec)

image_features_df = pd.DataFrame(encoded_images, columns=[f"img_feat_{i}" for i in range(512)])

conditioned_vars = pd.concat([conditioned_vars, image_features_df], axis=1)

conditioned_vars.to_csv("CVAE/data/conditioned_vars_with_img.csv", index=False)
