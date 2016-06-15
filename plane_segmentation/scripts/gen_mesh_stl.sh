echo "Genereting mesh"
time meshlabserver -i /tmp/obj.ply -o ./meshed.stl -s mesh_gen.mlx -om vc vn
echo "Done!"
