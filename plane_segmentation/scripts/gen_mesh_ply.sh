echo "Genereting mesh"
time meshlabserver -i /tmp/obj.ply -o ./meshed.ply -s mesh_gen.mlx -om vc vn
echo "Done!"
