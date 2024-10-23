aist_collision_object_manager
==================================================

## 概要
本パッケージは，[MoveIt](https://github.com/moveit/moveit)の[PlanningSceneInterface](https://github.com/moveit/moveit/blob/master/moveit_commander/src/moveit_commander/planning_scene_interface.py)を通じて[CollisionObject](http://docs.ros.org/en/noetic/api/moveit_msgs/html/msg/CollisionObject.html)および[AttachedCollisionObject](http://docs.ros.org/en/noetic/api/moveit_msgs/html/msg/AttachedCollisionObject.html)を操作することにより，ロボットがハンドリングする物体の接触状態を管理する機能を提供する．

## CollisionObjectとAttachedCollisionObjectの振る舞い
[MoveIt](https://github.com/moveit/moveit)の[CollisionObject](http://docs.ros.org/en/noetic/api/moveit_msgs/html/msg/CollisionObject.html)と[AttachedCollisionObject](http://docs.ros.org/en/noetic/api/moveit_msgs/html/msg/AttachedCollisionObject.html)は，次のように生成・消滅される．
- `CollisionObject`は，[PlanningSceneInterface.add_object()](https://github.com/moveit/moveit/blob/88b386581c5f25cc5733585bd39dfd2ea690329b/moveit_commander/src/moveit_commander/planning_scene_interface.py#L98)によって生成される．
- `CollisionObject`に[PlanningSceneInterface.attach_object()](https://github.com/moveit/moveit/blob/88b386581c5f25cc5733585bd39dfd2ea690329b/moveit_commander/src/moveit_commander/planning_scene_interface.py#L132)を適用すると`AttachedCollisionObject`に変容し，指定された`link`に接続される．`touch_links`を指定すれば，それも設定される(optional)．
- 既存の`AttachedCollisionObject`に対して`link`を指定せずに[PlanningSceneInterface.attach_object()](https://github.com/moveit/moveit/blob/88b386581c5f25cc5733585bd39dfd2ea690329b/moveit_commander/src/moveit_commander/planning_scene_interface.py#L132)を適用すると，`touch_links`のみが設定される．
- `AttachedCollisionObject`に対して[PlanningSceneInterface.remove_attached_object()](https://github.com/moveit/moveit/blob/88b386581c5f25cc5733585bd39dfd2ea690329b/moveit_commander/src/moveit_commander/planning_scene_interface.py#L197)を適用すると，リンクへの接続が解除され，このオブジェクトは`CollisionObject`に戻る．オブジェクトそのものが消滅するわけではない．
- `CollisionObject`に[PlanningSceneInterface.remove_world_object()](https://github.com/moveit/moveit/blob/88b386581c5f25cc5733585bd39dfd2ea690329b/moveit_commander/src/moveit_commander/planning_scene_interface.py#L187)を適用すると，このオブジェクトが消滅する．

また，以下の点に注意が必要である．

- [PlanningSceneInterface.add_object()](https://github.com/moveit/moveit/blob/88b386581c5f25cc5733585bd39dfd2ea690329b/moveit_commander/src/moveit_commander/planning_scene_interface.py#L98)によって生成される`CollisionObject`のポーズは，引数`collision_object`中の`header`と`pose`で指定される．`header.frame_id`には任意のフレームを指定できるが，生成後のポーズは`MoveIt`の`planning_frame`から見たものになる．すなわち，生成したオブジェクトの`id`を指定して[PlanningSceneInterface.get_objects()](https://github.com/moveit/moveit/blob/88b386581c5f25cc5733585bd39dfd2ea690329b/moveit_commander/src/moveit_commander/planning_scene_interface.py#L242)を呼ぶと，返される`CollisionObject`の`header.frame_id`は`planning_frame`(”world”など)となり，`pose`もこのフレームから見た値になる．
- `CollisionObject`を別`CollisionObject`や`AttachedCollisionObject`に接続することはできない．接続先のリンク(`attach_link`)は，`MoveIt`の起動時に`robot_description`から読み込まれるリンクツリーに含まれるものに限られる．
- [PlanningSceneInterface.get_objects()](https://github.com/moveit/moveit/blob/88b386581c5f25cc5733585bd39dfd2ea690329b/moveit_commander/src/moveit_commander/planning_scene_interface.py#L242)は，`CollisionObject`のみを返し，`AttachedCollisionObject`を返さない．
- [PlanningSceneInterface.get_attached_objects()](https://github.com/moveit/moveit/blob/88b386581c5f25cc5733585bd39dfd2ea690329b/moveit_commander/src/moveit_commander/planning_scene_interface.py#L254)は，`AttachedCollisionObject`のみを返し，`CollisionObject`を返さない．