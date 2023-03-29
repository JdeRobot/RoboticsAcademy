---
layout: splash
permalink: /
header:
#  overlay_color: "#5e616c"
  overlay_color: "#FFFFFF"
  overlay_image: /assets/images/cover/test_header_shear_3.png
#  actions:
#    - label: "<i class='fas fa-download'></i> Install now"
#      url: "/installation/"
excerpt: 
  A practical and fun way of learning Robotics and Computer Vision
  
feature_row:
  - image_path: /assets/images/cover/cover_column_1.png
    alt: "Exercises"
    title: "Exercises"
    excerpt: "Learn robot programming solving the Robotics Academy challenges"
    url: "/exercises/"
    btn_class: "btn--primary"
    btn_label: "Go!"

  - image_path: /assets/images/cover/cover_column_2.png
    alt: "fully responsive"
    title: "User Guide"
    excerpt: "Instructions for using Robotics Academy"
    url: "/user_guide"
    btn_class: "btn--primary"
    btn_label: "Go!"

#  - image_path: /assets/images/cover/cover_column_3.png
#    alt: "100% free"
#    title: "Web release (beta)"
#    excerpt: "Use Robotics Academy from your browser, no installation required"
#    url: "/webrelease"
#    btn_class: "btn--primary"
#    btn_label: "Learn more"

  - image_path: /assets/images/cover/forum1.png
    alt: "100% free"
    title: "Forum"
    excerpt: "Receive support, show your results and help others at the community forum"
    url: "https://forum.unibotics.org"
#    url: "/webrelease"
    btn_class: "btn--primary"
    btn_label: "Go!"

  - image_path: /assets/images/cover/cover_column_2.png
    alt: "fully responsive"
    title: "Contributor Guide"
    excerpt: "Info for RoboticsAcademy *developers*: create a new exercise, improve documentation, fix bugs.... Contributors are welcome!"
    url: "/developer_guide"
    btn_class: "btn--primary"
    btn_label: "Go!"

youTube_id: ID7qaEcIu4k
---


Robotics Academy is an **open source** collection of exercises and challenges to learn robotics in a practical way.
There are exercises about drone programming, about computer vision, about mobile robots, about autonomous cars, etc. 
It is mainly based on [Gazebo simulator](http://gazebosim.org) and [ROS](https://www.ros.org). The students program their solutions in **Python** language.

Each exercise is composed of (a) Gazebo configuration files, (b) a web template to host student's code and (c) theory contents. The students insert their code in the template file and use the provided simple API to access to _sensor readings_ and _actuator commands_ (HAL API) and the provided simple API for Graphical User Interface and debugging (GUI API).

For execution the student launches a Docker image containing a Gazebo simulation and connects to a Django webserver to insert his code.

{% include feature_row %}

{% include video id="Y6CoOK9WtDQ" provider="youtube" %}


# Scientific papers

1. *Automatic Competitions in the Unibotics open online robot programming web*.
Raúl Fernández-Ruiz, Daniel Palacios-Alonso, José Marı́a Cañas-Plaza, David Roldán-Álvarez. D. Tardioli et al. (Eds.): ROBOT 2022, LNNS 589, pp. 463–474, 2023. Springer Nature Switzerland AG. [DOI: https://doi.org/10.1007/978-3-031-21065-5_38](https://doi.org/10.1007/978-3-031-21065-5_38)

2. *A ROS-based open web platform for Intelligent Robotics education*. David Roldán, Sakshay Mahna José M. Cañas. International Conference on Robotics in Education (RiE-2021), pp 243-255, Advances in Intelligent Systems and Computing, vol 1359. Springer, 2022. [DOI: https://doi.org/10.1007/978-3-030-82544-7_23](https://doi.org/10.1007/978-3-030-82544-7_23)

3. *Open-Source Drone Programming Course for Distance Engineering Education*. José M. Cañas, Diego Martín-Martín, Pedro Arias, Julio Vega, David Roldán-Álvarez, Lía García-Pérez, Jesús Fernández-Conde. Electronics 2020, 9(12), 2163, MDPI 2020 (Special Issue Open Source Software in Learning Environments). [DOI: https://doi.org/10.3390/electronics9122163](https://doi.org/10.3390/electronics9122163)

4. *A ROS‐Based Open Tool for Intelligent Robotics Education*. José M. Cañas, Eduardo Perdices, Lía García-Pérez, Jesús Fernández-Conde. Applied Sciences 10(21), 7419, MDPI 2020 (Special Issue Advances in Artificial Intelligence Learning Technologies). [DOI: https://doi.org/10.3390/app10217419](https://doi.org/10.3390/app10217419)

5. *Entorno docente universitario para la programación de robots*. José M.Cañas, Alberto Martín, Eduardo Perdices, Francisco Rivas, Roberto Calvo. Revista Iberoamericana de Automática e Informática Industrial, 15, 404-415, 2018 [DOI: https://doi.org/10.4995/riai.2018.8962](https://doi.org/10.4995/riai.2018.8962)

# Sponsors

<figure class="third">
    <a href="https://www.urjc.es/" target="_blank"><img src="{{ site.url }}{{ site.baseurl }}/assets/images/cover/logoURJC.jpg" style="width:400px;"></a>
    <a href="https://github.com/RoboticsLabURJC" target="_blank"><img src="{{ site.url }}{{ site.baseurl }}/assets/images/cover/peloto.png" style="width:400px;"></a>
    <a href="https://summerofcode.withgoogle.com" target="_blank"><img src="{{ site.url }}{{ site.baseurl }}/assets/images/cover/gsoc.png" style="width:200px;"></a>
</figure>
