from PIL import Image
import os.path
import glob

def convertjpg(jpgfile,outdir,width=512,height=512):
  img=Image.open(jpgfile)
  try:
    new_img=img.resize((width,height),Image.BILINEAR)
    if not os.path.exists(outdir):
        os.mkdir(outdir)
    new_img.save(os.path.join(outdir,os.path.basename(jpgfile)))
  except Exception as e:
    print(e)

path = r"D:\liti\shuangmuceju\tupian\you\*.png"
for jpgfile in glob.glob(path):
  convertjpg(jpgfile,r"D:\liti\shuangmuceju\tupian\you1")
