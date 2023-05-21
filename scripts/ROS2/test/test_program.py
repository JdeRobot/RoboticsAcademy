from hal import HAL
curro = HAL()


curro.setW(0.5)


while True:
    curro.setV(0)
    img = curro.getImage()
    print(img.shape)
