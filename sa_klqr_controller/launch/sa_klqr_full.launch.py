<launch>

    <!-- Load controller parameters -->
    <param name="sa_klqr_controller/controller_config"
           value="$(find sa_klqr_controller)/config/controller.yaml" />

    <!-- Run SA-KLQR node -->
    <node pkg="sa_klqr_controller" type="sa_klqr_node.py" name="sa_klqr_node" output="screen">
        <param name="controller_config" 
               value="$(find sa_klqr_controller)/config/controller.yaml" />
    </node>

</launch>
