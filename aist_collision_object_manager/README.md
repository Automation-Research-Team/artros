aist_collision_object_manager
==================================================

## 概要
本パッケージは，[MoveIt](https://github.com/moveit/moveit)の[PlanningSceneInterface](https://github.com/moveit/moveit/blob/master/moveit_commander/src/moveit_commander/planning_scene_interface.py)を通じて[CollisionObject](http://docs.ros.org/en/noetic/api/moveit_msgs/html/msg/CollisionObject.html)および[AttachedCollisionObject](http://docs.ros.org/en/noetic/api/moveit_msgs/html/msg/AttachedCollisionObject.html)を操作することにより，ロボットがハンドリングする物体の接触状態を管理する機能を提供する．

## CollisionObjectとAttachedCollisionObjectの振る舞い
[CollisionObject](http://docs.ros.org/en/noetic/api/moveit_msgs/html/msg/CollisionObject.html)と[AttachedCollisionObject](http://docs.ros.org/en/noetic/api/moveit_msgs/html/msg/AttachedCollisionObject.html)は，次のように生成・消滅する．
- `CollisionObject`は，[add_object()](https://github.com/moveit/moveit/blob/88b386581c5f25cc5733585bd39dfd2ea690329b/moveit_commander/src/moveit_commander/planning_scene_interface.py#L98)によって生成される．
- `CollisionObject`に[attach_object()](https://github.com/moveit/moveit/blob/88b386581c5f25cc5733585bd39dfd2ea690329b/moveit_commander/src/moveit_commander/planning_scene_interface.py#L132)を適用すると`AttachedCollisionObject`に変容し，指定された`link`に接続される．`touch_links`を指定すれば，それも設定される(optional)．
- 既存の`AttachedCollisionObject`に対して`link`を指定せずに[attach_object()](https://github.com/moveit/moveit/blob/88b386581c5f25cc5733585bd39dfd2ea690329b/moveit_commander/src/moveit_commander/planning_scene_interface.py#L132)を適用すると，`touch_links`のみが設定される．
- `AttachedCollisionObject`に対して[remove_attached_object()](https://github.com/moveit/moveit/blob/88b386581c5f25cc5733585bd39dfd2ea690329b/moveit_commander/src/moveit_commander/planning_scene_interface.py#L197)を適用すると，リンクへの接続が解除され，このオブジェクトは`CollisionObject`に戻る．オブジェクトそのものが消滅するわけではない．
- `CollisionObject`に[remove_world_object()](https://github.com/moveit/moveit/blob/88b386581c5f25cc5733585bd39dfd2ea690329b/moveit_commander/src/moveit_commander/planning_scene_interface.py#L187)を適用すると，このオブジェクトが消滅する．

また，`PlanningSceneInterface`のAPIに関して，以下の点に注意が必要である．

- [add_object()](https://github.com/moveit/moveit/blob/88b386581c5f25cc5733585bd39dfd2ea690329b/moveit_commander/src/moveit_commander/planning_scene_interface.py#L98)によって生成される`CollisionObject`のポーズは，引数`collision_object`中の`header`と`pose`で指定される．`header.frame_id`には任意のフレームを指定できるが，生成後のポーズは`MoveIt`の`planning_frame`から見たものになる．すなわち，生成したオブジェクトの`id`を指定して[get_objects()](https://github.com/moveit/moveit/blob/88b386581c5f25cc5733585bd39dfd2ea690329b/moveit_commander/src/moveit_commander/planning_scene_interface.py#L242)を呼ぶと，返される`CollisionObject`の`header.frame_id`は`planning_frame`(例えば”world”)となり，`pose`もこのフレームから見た値になる．
- `CollisionObject`を別の`CollisionObject`や`AttachedCollisionObject`に接続することはできない．接続先のリンク(`attach_link`)は，`MoveIt`の起動時に`robot_description`から読み込まれるリンクツリーに含まれるものに限られる．
- [get_objects()](https://github.com/moveit/moveit/blob/88b386581c5f25cc5733585bd39dfd2ea690329b/moveit_commander/src/moveit_commander/planning_scene_interface.py#L242)は，`CollisionObject`のみを返し，`AttachedCollisionObject`を返さない．
- [get_attached_objects()](https://github.com/moveit/moveit/blob/88b386581c5f25cc5733585bd39dfd2ea690329b/moveit_commander/src/moveit_commander/planning_scene_interface.py#L254)は，`AttachedCollisionObject`のみを返し，`CollisionObject`を返さない．

## ハンドリング対象物体の表現
対象物体は[aist_descriptions/parts](../aist_description/parts)で用いられる形式に準拠して表現される．具体的には，[parts_properties.yaml](../aist_description/parts/config/parts_properties.yaml)を例とするYAMLファイルによってその幾何形状，衝突形状，サブフレーム等が指定される．`collision_object_manager`は，起動時にこれを読み込んで全対象物体の情報を内部に保持する．

各サブフレームは，`MoveIt`から`<object ID>/<subframe name>`というフレーム名で見える．しかし，`MoveIt`の外部からは見えないので，`collision_object_manager`は，こ
れらを同名で`tf`にbroadcastすることによって，どこからでも見えるようにしている．

YAMLファイルに指定されたサブフレームの他に，対象物体はそのポーズを指定するためのフレームを持ち，`MoveIt`から`<object ID>/base_link`というフレーム名で見える．各サブフレームのポーズは，このフレームから見たものである．これも同名で`tf`にbroadcastされている．

## ハンドリング対象物体間の接触・接続関係の表現
組立作業においては，部品間の接触・接続関係やグリッパで部品を把持したときの接続関係を表現し，経路計画の干渉判定に際してシーンに存在する部品が考慮される必要がある．