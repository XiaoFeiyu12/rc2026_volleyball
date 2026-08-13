# 方便每次改代码调参数编译使用
docker exec -it volleyball-robot bash -c "colcon build --parallel-workers 4"
docker compose restart volleyball
