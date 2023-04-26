function updateRenderer(image) {
  this.clearRect(0, 0, image.width, image.height);
  this.drawImage(image, 0, 0, image.width, image.height);
}

export default updateRenderer;