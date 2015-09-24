uint collResolvedPB=true; //Package-Boot
uint collResolvedPP=true; //Package-Package
srand(time(NULL));
uint k=0;
bool jumpRotation=false;
while(collResolvedPB+collResolvedPP>0){//(collResolvedPB+collResolvedPP>1) && k<50
    collResolvedPB=0;
    collResolvedPP=0;
    uint n=0;

    if(k%30==0) jumpRotation=true;

    k++;


    int CGView::resolveCollision(Package &box, BVT &off, bool jumpRot){
        this->collDir=Vector3d(1,0,0)*1e300;

        bool coll_occ=off.intersect(box);;
        uint j=0;
        vecvec3d potDir,triMids;
        Vector3d move_dir=0,rotDir=0;

        move_dir=box.packageInBox(off.getBox());
        if(move_dir.length()!=0)
            coll_occ=true;

        while(coll_occ && j<5){
            ++j;
            off.getIntersectDirs(potDir,triMids);
            move_dir=0;
            for (uint i = 0; i < potDir.size(); ++i) {
                move_dir+=potDir[i];
                rotDir+=((box.getCenter()-triMids[i])%potDir[i]);
                move_dir=move_dir/potDir.size()*0.9;
                rotDir=rotDir/potDir.size();
                Quat4d rot=Quat4d(0.01,rotDir);

                if(move_dir.length()<box.getDiameter()+off.getBox().getDiameter()){
                    if(move_dir.lengthSquared()<1e-10)
                        move_dir=move_dir.normalized()*std::abs(box.getCenter().minComp())*1e-6;
                    box.move(move_dir*0.01);
                    //covers case rot=nan:
                    if(rot.length()==rot.length())
                    box.rotate(rot);
                }
                move_dir=box.packageInBox(off.getBox());
        //        if(move_dir.length()!=0)
        //            coll_occ=true;
                box.move(move_dir*0.01);
             }
        return potDir.size();
    }
