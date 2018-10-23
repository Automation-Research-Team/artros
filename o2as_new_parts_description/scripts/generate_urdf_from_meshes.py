#!/usr/bin/python

import csv
import os

import rospy
import rospkg
rp = rospkg.RosPack()

print("D1")

# Read in the files
filenames = os.listdir(os.path.join(rp.get_path("o2as_new_parts_description"), "meshes"))
filenames_strip1 = []
filenames_no_ext = []
for name in filenames:
    filenames_strip1.append(os.path.splitext(name)[0]) 
for name in filenames_strip1: 
    filenames_no_ext.append(os.path.splitext(name)[0])        # This removes the .vhacd from ".vhacd.dae" files
partnames = list(sorted(set(filenames_no_ext)))   # Removes duplicates and sorts (because sets do not allow duplicate entries)
print("D1")
out_dir = os.path.join(rp.get_path("o2as_new_parts_description"), "urdf/generated")

# Read in the templates
extra_joint_filename = os.path.join(rp.get_path("o2as_new_parts_description"), "urdf/templates", "extra_frames.csv")
macro_template_filename = os.path.join(rp.get_path("o2as_new_parts_description"), "urdf/templates", "macro_template.urdf.xacro")
f = open(macro_template_filename,'r')
macro_template = f.read()
f.close()

macro_frames_only_template_filename = os.path.join(rp.get_path("o2as_new_parts_description"), "urdf/templates", "macro_frames_only_template.urdf.xacro")
f = open(macro_frames_only_template_filename,'r')
macro_frames_only_template = f.read()
f.close()

non_macro_template_filename = os.path.join(rp.get_path("o2as_new_parts_description"), "urdf/templates", "non_macro_template.urdf.xacro")
f = open(non_macro_template_filename,'r')
non_macro_template = f.read()
f.close()

spawn_template_filename = os.path.join(rp.get_path("o2as_new_parts_description"), "urdf/templates", "spawn_template.urdf")
f = open(spawn_template_filename,'r')
spawn_template = f.read()
f.close()
print("Reading")
extra_frames = []
with open(extra_joint_filename, 'r') as f:
  reader = csv.reader(f)
  header = next(reader)
  for row in reader:
    row_stripped = []
    for el in row:
      row_stripped.append(el.strip())       # Removes whitespaces
    extra_frames.append(row_stripped)

# Write the spawn files
# --- This is ignored for now

# Write the macros
if not os.path.exists(out_dir):
    os.makedirs(out_dir)
for part_num, partname in enumerate(partnames):
    print("Writing partname")
    macrofile = open(os.path.join(out_dir, partname+"_macro.urdf.xacro"),'w+')
    macro_frames_only_file = open(os.path.join(out_dir, partname+"_frames_only_macro.urdf.xacro"),'w+')

    mname_int = "assy_part_" + partname[0:2]                    # = "assy_part_01" for part 01.
    mname_ext = mname_int
    mname_fo_int = mname_int
    mname_fo_ext = "assy_part_frames_only_" + partname[0:2]     # This only changes the name of the macro, not the joint/link names
    macrofile_content = macro_template.replace("PARTNAME", partname)
    macrofile_content = macrofile_content.replace("MACRONAME_INTERNAL", mname_int)
    macrofile_content = macrofile_content.replace("MACRONAME_EXTERNAL", mname_ext)
    macro_frames_only_filecontent = macro_frames_only_template.replace("MACRONAME_INTERNAL", mname_fo_int)
    macro_frames_only_filecontent = macro_frames_only_filecontent.replace("MACRONAME_EXTERNAL", mname_fo_ext)

    if int(partname[0:2]) in [1,2,3,4]:
        macrofile_content = macrofile_content.replace("vhacd.dae", "stl")
        macrofile_content = macrofile_content.replace("dae", "stl")

    # if int(partname[0:2]) == 4:
    #     macrofile_content = macrofile_content.replace("0.001", "0.000001")      # To correct for that mesh's scale

    extra_frames_urdf = ""
    for entry in extra_frames:
        if int(entry[0]) == int(partname[0:2]):
            new_joint = ""
            new_joint +=  "    <joint name=\"${prefix}" + mname_int + "_LINK_NAME_joint\" type=\"fixed\"> \n"
            new_joint +=  "      <parent link=\"${prefix}" + mname_int + "\"/> \n"
            new_joint +=  "      <child link=\"${prefix}" + mname_int + "_LINK_NAME\"/> \n"
            new_joint +=  "      <origin rpy=\"${" + \
                                entry[2] + "} ${" + \
                                entry[3] + "} ${" + \
                                entry[4] + "}\" xyz=\"${" + \
                                entry[5] + "} ${" + \
                                entry[6] + "} ${" + \
                                entry[7] + "}\"/> \n"
            new_joint +=  "    </joint> \n"
            new_joint +=  "    <link name=\"${prefix}" + mname_int + "_LINK_NAME\"/> \n"
            new_joint +=  "    \n"
            new_joint = new_joint.replace("LINK_NAME", entry[1])
            extra_frames_urdf += new_joint
    if extra_frames_urdf:
        macrofile_content = macrofile_content.replace("<!-- #EXTRAFRAMES -->", extra_frames_urdf)
        macro_frames_only_filecontent = macro_frames_only_filecontent.replace("<!-- #EXTRAFRAMES -->", extra_frames_urdf)
    macrofile.write(macrofile_content)
    macro_frames_only_file.write(macro_frames_only_filecontent)
    print("Wrote " + os.path.join(out_dir, partname+"_macro.urdf.xacro"))
    print("Wrote " + os.path.join(out_dir, partname+"_frames_only_macro.urdf.xacro"))

for part_num, partname in enumerate(partnames):
    non_macrofile = open(os.path.join(out_dir, partname+"_non_macro.urdf.xacro"),'w+')
    mname_ext = "assy_part_" + partname[0:2]
    non_macrofile_content = non_macro_template.replace("MACRONAME_EXTERNAL", mname_ext)
    non_macrofile_content = non_macrofile_content.replace("PARTNAME", partname)
    non_macrofile.write(non_macrofile_content)
    print("Wrote " + os.path.join(out_dir, partname+"_macro.urdf.xacro"))


# Convert xacro files to urdf (necessary for the URDF-to-msg converter)
import subprocess
for part_num, partname in enumerate(partnames):
    print("Convert xacro to urdf: {}".format(partname))
    non_macrofilepath = os.path.join( out_dir, partname+"_non_macro.urdf.xacro")
    out_urdf_filepath = os.path.join( os.path.join(out_dir, "collision_object_urdfs"), partname+"_non_macro.urdf")
    cmd = 'xacro --inorder ' + non_macrofilepath + " -o " + out_urdf_filepath
    subprocess.check_call(cmd, shell=True)
