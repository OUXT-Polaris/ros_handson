target "default" {
  dockerfile = "Dockerfile"
  tags = ["ros_handson"]
  platforms = ["linux/amd64", "linux/arm64"]
}