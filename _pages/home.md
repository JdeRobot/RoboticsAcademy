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
  
basic_row:
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


Robotics Academy is an **open source** collection of exercises and challenges to learn robotics in a practical way. It includes exercises about mobile robotics, service robotics, autonomous driving, drones, computer vision in robotics, etc. (Check the [RoboticsAcademy playlist](https://www.youtube.com/playlist?list=PLGlX46StCA-TgY83tjwzEC1WodX2m-Eoe) for watching illustrative videos). It is mainly based on [Gazebo simulator](http://gazebosim.org) and [ROS](https://www.ros.org). The students program their solutions in **Python** language.

Each exercise is composed of (a) Gazebo configuration files, (b) a webpage and (c) the user documentation with theory contents and hints. The students can edit, debug and run their robotics applications from the browser. Two auxiliary Python modules are provided for each exercise and may be (optionally) used: *HAL.py* for easy access to _sensor readings_ and _actuator commands_ and *WebGUI.py* for Web Graphical User Interface at the browser. 

For successful *offline execution* the student launches a Docker image (named RADI, RoboticsAcademy Docker Image) containing all the robotics dependencies (ROS2 Humble, Gazebo simulator, OpenCV, PyTorch...) already preinstalled and then connects to a Django webserver running inside RADI from the browser to land in the corresponding exercise webpage. Alternatively it can also be used online at [Unibotics](https://unibotics.org), the JdeRobot robot programming website.

{% include basic_row %}

{% include video id="XzgfaQ20atY" provider="youtube" %}


# Scientific papers

1. *Teaching Service Robotics with ROS and Unibotics web framework in Higher Education*. Lucı́a Chen, José M. Cañas, David Roldán, Diego Martı́n, Lı́a Garcı́a-Perez, Florian Stöckl, Silvan Müller and Marcus Strand. International Conference on Robotics in Education (RiE-2025) Apr 23-25, Thessaloniki (Greece). Springer Nature Switzerland.

2. *Gamification in a Web-based platform for teaching robotics engineering*. David Rodrı́guez-Rives, Raúl Fernández-Ruiz, Daniel Palacios-Alonso, Nikola Hristov-Kalamov and José M. Cañas
International Conference on Robotics in Education (RiE-2025) Apr 23-25, Thessaloniki (Greece). Springer Nature Switzerland. 

3. *Introduction to Control Education with the Unibotics web framework*. Lía García-Pérez, Diego Martín-Martín, José M. Cañas, Jesús Chacón, David Roldán. XLV Jornadas de Automática, 2024. [DOI: https://doi.org/10.17979/ja-cea.2024.45.10933](https://doi.org/10.17979/ja-cea.2024.45.10933)

4. *Automatic code assessment in Robotics higher education courses*. Lı́a Garcı́a-Pérez, José M. Cañas, David Roldán. 10th International Conference on Higher Education Advances (HEAd-24), June 18-21, Valencia, (pp.1263-1270). [DOI: https://doi.org/10.4995/HEAd24.2024.17362](DOI: https://doi.org/10.4995/HEAd24.2024.17362)

5. *Improving usability of a Web-based platform for teaching robotics engineering*. Lı́a Garcı́a-Pérez, David Roldán, Enric Cervera, Pawan Wadhwani, José M. Cañas. International Conference on Robotics in Education (RiE-2024) Apr 10, Koblenz (Germany), (pp. 313-324). Springer Nature Switzerland. [DOI: https://doi.org/10.1007/978-3-031-67059-6_28](DOI: https://doi.org/10.1007/978-3-031-67059-6_28)

6. *Unibotics: open ROS-based online framework for practical learning of Robotics in higher education*. David Roldán-Álvarez, José M. Cañas, David Valladares, Pedro Arias-Perez, Sakshay Mahna. Multimedia Tools and Applications, Springer 2023. [DOI: https://doi.org/10.1007/s11042-023-17514-z](https://doi.org/10.1007/s11042-023-17514-z)

7. *Automatic Competitions in the Unibotics open online robot programming web*.
Raúl Fernández-Ruiz, Daniel Palacios-Alonso, José Marı́a Cañas-Plaza, David Roldán-Álvarez. D. Tardioli et al. (Eds.): ROBOT 2022, LNNS 589, pp. 463–474, 2023. Springer Nature Switzerland AG. [DOI: https://doi.org/10.1007/978-3-031-21065-5_38](https://doi.org/10.1007/978-3-031-21065-5_38)

8. *A ROS-based open web platform for Intelligent Robotics education*. David Roldán, Sakshay Mahna José M. Cañas. International Conference on Robotics in Education (RiE-2021), pp 243-255, Advances in Intelligent Systems and Computing, vol 1359. Springer, 2022. [DOI: https://doi.org/10.1007/978-3-030-82544-7_23](https://doi.org/10.1007/978-3-030-82544-7_23)

9. *Open-Source Drone Programming Course for Distance Engineering Education*. José M. Cañas, Diego Martín-Martín, Pedro Arias, Julio Vega, David Roldán-Álvarez, Lía García-Pérez, Jesús Fernández-Conde. Electronics 2020, 9(12), 2163, MDPI 2020 (Special Issue Open Source Software in Learning Environments). [DOI: https://doi.org/10.3390/electronics9122163](https://doi.org/10.3390/electronics9122163)

10. *A ROS‐Based Open Tool for Intelligent Robotics Education*. José M. Cañas, Eduardo Perdices, Lía García-Pérez, Jesús Fernández-Conde. Applied Sciences 10(21), 7419, MDPI 2020 (Special Issue Advances in Artificial Intelligence Learning Technologies). [DOI: https://doi.org/10.3390/app10217419](https://doi.org/10.3390/app10217419)

11. *Entorno docente universitario para la programación de robots*. José M.Cañas, Alberto Martín, Eduardo Perdices, Francisco Rivas, Roberto Calvo. Revista Iberoamericana de Automática e Informática Industrial, 15, 404-415, 2018 [DOI: https://doi.org/10.4995/riai.2018.8962](https://doi.org/10.4995/riai.2018.8962)

# Sponsors

<figure class="third">
    <a href="https://www.urjc.es/" target="_blank"><img src="{{ site.url }}{{ site.baseurl }}/assets/images/cover/logoURJC.jpg" style="width:400px;"></a>
    <a href="https://github.com/RoboticsLabURJC" target="_blank"><img src="{{ site.url }}{{ site.baseurl }}/assets/images/cover/peloto.png" style="width:400px;"></a>
    <a href="https://summerofcode.withgoogle.com" target="_blank"><img src="{{ site.url }}{{ site.baseurl }}/assets/images/cover/gsoc.png" style="width:200px;"></a>
</figure>
