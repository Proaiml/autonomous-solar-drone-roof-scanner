import torch
import torch.nn as nn
from torchvision import transforms,models,datasets
from torch.utils.data import DataLoader, random_split
import time
import warnings
import matplotlib.pyplot as plt

warnings.filterwarnings(
    "ignore",
    message="Palette images with Transparency expressed in bytes should be converted to RGBA images"
)
def main():

    transform = transforms.Compose([
        transforms.Resize((224,224)),
        transforms.ToTensor(),
    transforms.Normalize(
        mean=[0.485, 0.456, 0.406],
        std=[0.229, 0.224, 0.225]
    )
    ])

    dataset = datasets.ImageFolder(
        "C:/Users/İlhan/Desktop/ödev3/Detect_solar_dust",
        transform=transform
    )

    print(dataset.classes)
    print(len(dataset))
    print(dataset.class_to_idx)

    train_size = (len(dataset) * 80 // 100)
    test_size = (len(dataset)  - train_size )
    print(train_size,test_size)

    train_dataset , test_dataset = random_split(dataset, [train_size,test_size])

    train_dataloader = DataLoader(train_dataset,batch_size=64,shuffle=True,num_workers=10,pin_memory=True, persistent_workers=True,prefetch_factor=4,drop_last=True)
    test_dataloader = DataLoader(test_dataset,batch_size=64,shuffle=False,num_workers=10,pin_memory=True, persistent_workers=True,prefetch_factor=4)

    model = models.resnet18(weights = models.ResNet18_Weights.DEFAULT)
    print(model)

    for param in model.parameters():
        param.requires_grad = False

    model.fc = model.fc = nn.Sequential(
    nn.Linear(model.fc.in_features,50),
    nn.ReLU(),
    nn.Dropout(0.3),

    nn.Linear(50,20),
    nn.ReLU(),
    nn.Dropout(0.3),

    nn.Linear(20,10),
    nn.ReLU(),
    nn.Dropout(0.3),

    nn.Linear(10,2)
)

    print(model)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = model.to(device)

    print(device)

    criterion = nn.CrossEntropyLoss()
    optimizer=torch.optim.Adam(params=model.fc.parameters(),lr=0.001,weight_decay=1e-4)
    train_losses = []
    test_losses = []


    for epoch in range(20):
        start = time.time()
        model.eval()
        model.fc.train()
        print(epoch)
        train_loss = 0.0
        for images,labels in train_dataloader:
            image = images.to(device, non_blocking=True)
            labels = labels.to(device, non_blocking=True)
            optimizer.zero_grad()
            pred = model(image)
            loss  = criterion(pred,labels)
            train_loss += loss.item() * images.size(0)


            loss.backward()
            optimizer.step()
        print("Epoch:", epoch, "Süre:", time.time() - start)
        train_loss = train_loss / len(train_dataset)
        train_losses.append(train_loss)

        model.eval()

        test_loss = 0.0

        with torch.no_grad():
            for images, labels in test_dataloader:
                images = images.to(device, non_blocking=True)
                labels = labels.to(device, non_blocking=True)

                pred = model(images)
                loss = criterion(pred, labels)

                test_loss += loss.item() * images.size(0)

        test_loss = test_loss / len(test_dataset)
        test_losses.append(test_loss)
    torch.save(
        model.state_dict(),
        "C:/Users/İlhan/Desktop/ödev3/resnet18_solar_dust.pth"
    )
    plt.legend(["Train Loss", "Test Loss"])
    plt.xlabel("Epoch")
    plt.ylabel("Loss")
    plt.plot(train_losses)
    plt.plot(test_losses)
    plt.show()

if __name__ == "__main__":
    main()