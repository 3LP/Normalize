
/*
 * $Copyright$
 * Copyright (c) 2015 All Rights Reserved, Sean Harre
 *
 * Sean Harre is the copyright holder of all code below.
 * Do not re-use without permission.
 */

#include "leapDriver.h"

// this file interfaces to leap motion and
// handles drawing hands

// standalone function to instantiate a singleton threadWorker
leapDriver& getLeapDrv()
{
    static leapDriver* ldPtr = 0;
    if (!ldPtr) {
        cout<< "create instance of leapDriver" <<endl;
        ldPtr = new leapDriver;
    }
    return *ldPtr;
}

//--------------------------
leapDriver::leapDriver() :
    rfound(0),lfound(0),debug_mode(false),
    do_leap_filtering(true)
{
    set_hand_filtering(do_leap_filtering);
}

//--------------------------
leapDriver::~leapDriver()
{
    (void)stop();
}

//--------------------------
bool leapDriver::set_debug_mode(bool newv)
{
    bool was = debug_mode;
    debug_mode = newv;
    return was;
}

//--------------------------
bool leapDriver::is_this_special_thumb_joint(int finger, int joint)
{
    return ((finger==0) && (joint==1));
}

//--------------------------
bool leapDriver::start(const Leap::Controller& controller) {
    
#if CFG_HAS_LEAP_CTRL
    dbprintf("Init Leap...");
    // timeout @ 1sec if not ready, normally takes ~150ms
    for (int i=0; i<100; ++i)
    {
        volatile Leap::Frame frame = controller.frame();
        if (controller.isConnected())
        {
            dbprintf("SUCCESS\n");
            
            // default idx values
            for (int i=0; i < 2; ++i)
            {
                Leap::Hand hand = controller.frame().hands()[i];
                if (hand.isLeft())
                    lhand = deep_copy_hand(hand,true);
                else
                    rhand = deep_copy_hand(hand,false);
            }
            
            lhand.is_valid = false;
            rhand.is_valid = false;
            
            return true;
        }
        usleep(100000);  // 100ms sleep
        dbprintf(".");
    }
    dbprintf("FAIL\n");
    return false;
#else
    return true;
#endif
}

//--------------------------
bool leapDriver::stop() {
    return true;
}

//--------------------------
void leapDriver::set_hand_filtering(bool set_on)
{
    do_leap_filtering = set_on;
    if (set_on)
        disappear_count = LEAP_TIME_NO_HAND_DISAPPEAR_FILT;
    else
        disappear_count = LEAP_TIME_NO_HAND_DISAPPEAR_NOFILT;
}

//--------------------------
bool leapDriver::get_hand_filtering()
{
    return do_leap_filtering;
}

//--------------------------
// filter:
//  m = coeff*(1-confidence)
//  y = x0*m + x1*(1-m)
//
// as confidence increases, use more newv, as confidence falls, use more oldv
Leap::Vector leapDriver::flt(const Leap::Vector newv, const Leap::Vector oldv, const float coeff, const float confid) {
    if (!do_leap_filtering)
    {
        return newv;
    }
    else
    {
        float m = coeff * (1.0f - confid);
        Leap::Vector r = (oldv * m) + (newv * (1.0f-m));
        return r;
    }
}

//--------------------------
void leapDriver::clear_impulse_data(HandVector::hand_vecs& hand)
{
    for (int f=0; f<MAX_FINGERS; ++f)
        for (int j=0; j<MAX_JOINTS; ++j)
        {
            hand.fingers[f].joints[j].collision_force = 0.0f;
            hand.fingers[f].joints[j].depth = 0.0f;
        }
}

//--------------------------
HandVector::hand_vecs leapDriver::deep_copy_hand(const Leap::Hand& hand, bool isLeft)
{
    dbprintf(" *** DEEP COPY %s ***\n",isLeft ? "LEFT" : "RIGHT");
    
    HandVector::hand_vecs tmp;
    ktObjectInfo lcl;
    lcl.is_left = isLeft;
    lcl.shape_type = KT_PALM;
    
    if (hand.isValid())
        copy_one_joint_vecs(tmp.palm_location, hand.stabilizedPalmPosition(), 0, 0, lcl, 0, 0, 0, ktQuaternion(0,0,0,0));
    else
        copy_one_joint_vecs(tmp.palm_location, Leap::Vector(JOINT_UNDEF.x,LEAP_JOINT_UNDEF.y,LEAP_JOINT_UNDEF.z), 0, 0, lcl, 0, 0, 0, ktQuaternion(0,0,0,0));
    
    int jj = 0;
    for (int f = 0; f < MAX_FINGERS; ++f) {
        Leap::Finger finger = hand.fingers()[f];
        
        if ( finger.isValid() )
        {
            // first joint inside hand, wrist joint
            Leap::Bone mcp = finger.bone(Leap::Bone::TYPE_METACARPAL);
            
            copy_one_joint_vecs(tmp.fingers[f].joints[Leap::Bone::TYPE_METACARPAL],
                                mcp.center(), 0, 0, gen_and_stuff_UserInfo(f,0,isLeft), jj++,
                                mcp.width(), mcp.length(), LeapBasis_to_ktQuaternion(mcp.basis()));
            
            for (int b = 0; b < MAX_BONES; ++b) {
                
                Leap::Bone bone = finger.bone(static_cast<Leap::Bone::Type>(b));
                
                // is_this_special_thumb_joint(f,b+1)
                if ((f==0) && (b==0))
                {
                    // this is "special" thumb extra bone w/ zero length, make it same as wrist joint
                    copy_one_joint_vecs(tmp.fingers[f].joints[b+1],
                                        tmp.fingers[f].joints[Leap::Bone::TYPE_METACARPAL].pos,
                                        tmp.fingers[f].joints[Leap::Bone::TYPE_METACARPAL].collision_force,
                                        tmp.fingers[f].joints[Leap::Bone::TYPE_METACARPAL].depth,
                                        tmp.fingers[f].joints[Leap::Bone::TYPE_METACARPAL].idx,
                                        tmp.fingers[f].joints[Leap::Bone::TYPE_METACARPAL].ftdi_idx,
                                        mcp.width(), mcp.length(), LeapBasis_to_ktQuaternion(mcp.basis()));
                }
                else
                {
                    copy_one_joint_vecs(tmp.fingers[f].joints[b+1], bone.nextJoint(), 0, 0, gen_and_stuff_UserInfo(f,(b+1),isLeft), jj++, mcp.width(), mcp.length(), LeapBasis_to_ktQuaternion(mcp.basis()));
                }
            }
        }
        else
        {
            // default values
            
            // first joint inside hand, wrist joint
            copy_one_joint_vecs(tmp.fingers[f].joints[Leap::Bone::TYPE_METACARPAL],
                                LEAP_JOINT_UNDEF, 0, 0, gen_and_stuff_UserInfo(f,0,isLeft), jj++,
                                 0, 0, ktQuaternion(0,0,0,0));
            
            for (int b = 0; b < MAX_BONES; ++b) {
                
                // // is_this_special_thumb_joint(f,b+1)
                if ((f==0) && (b==0))
                {
                    // this is "special" thumb extra bone w/ zero length, make it same as wrist joint
                    copy_one_joint_vecs(tmp.fingers[f].joints[b+1],
                                        tmp.fingers[f].joints[Leap::Bone::TYPE_METACARPAL].pos,
                                        tmp.fingers[f].joints[Leap::Bone::TYPE_METACARPAL].collision_force,
                                        tmp.fingers[f].joints[Leap::Bone::TYPE_METACARPAL].depth,
                                        tmp.fingers[f].joints[Leap::Bone::TYPE_METACARPAL].idx,
                                        tmp.fingers[f].joints[Leap::Bone::TYPE_METACARPAL].ftdi_idx,
                                         0, 0, ktQuaternion(0,0,0,0));
                }
                else
                {
                    copy_one_joint_vecs(tmp.fingers[f].joints[b+1], LEAP_JOINT_UNDEF, 0, 0, gen_and_stuff_UserInfo(f,(b+1),isLeft), jj++,  0, 0, ktQuaternion(0,0,0,0));
                }
            }
        }
    }
    
    return tmp;
}

//
// From: https://community.leapmotion.com/t/how-can-i-get-the-quaternions-out-of-the-bone-basis/1496
//
//ci::Matrix44f derivedRotMatrix = LeapMotion::toMatrix44f(bone.basis().rigidInverse());
//float w = sqrtf(1.0 + derivedRotMatrix.at(0,0) + derivedRotMatrix.at(1,1) + derivedRotMatrix.at(2,2)) / 2.0;
//double w4 = (4.0 * w);
//float x = (derivedRotMatrix.at(2,1) - derivedRotMatrix.at(1,2)) / w4 ;
//float y = (derivedRotMatrix.at(0,2) - derivedRotMatrix.at(2,0)) / w4 ;
//float z = (derivedRotMatrix.at(1,0) - derivedRotMatrix.at(0,1)) / w4 ;
//ci::Quatf rotation(w, x, y , z);
//
ktQuaternion leapDriver::LeapBasis_to_ktQuaternion(Leap::Matrix basis)
{
    float matrix_values[16];
    basis.rigidInverse().toArray4x4(matrix_values);
    float w = sqrtf(1.0 + matrix_values[0,0] + matrix_values[1,1] + matrix_values[2,2]) / 2.0;
    double w4 = (4.0 * w);
    float x = (matrix_values[2,1] - matrix_values[1,2]) / w4 ;
    float y = (matrix_values[0,2] - matrix_values[2,0]) / w4 ;
    float z = (matrix_values[1,0] - matrix_values[0,1]) / w4 ;
    ktQuaternion rotation(x, y , z, w);
    return rotation;
}

//--------------------------
HandVector::hand_vecs leapDriver::get_right_hand_filtered(const Leap::Controller& controller) {
    
    bool saw = false;
    Leap::Frame frame = controller.frame();
    for (int h = 0; h < frame.hands().count(); ++h) {
        
        Leap::Hand hand = frame.hands()[h];
        if (hand.isRight())
        {
            if ( ! rfound)
            {
                rhand = deep_copy_hand(hand,false);
                rhand.is_valid = true;
            }
            else
            {
                rhand.palm_location.pos = hand.stabilizedPalmPosition();
                
                for (int f = 0; f < hand.fingers().count(); ++f) {
                    Leap::Finger finger = hand.fingers()[f];
                    
                    // first joint inside hand.
                    Leap::Bone mcp = finger.bone(Leap::Bone::TYPE_METACARPAL);
                    rhand.fingers[f].joints[Leap::Bone::TYPE_METACARPAL].pos = flt(mcp.center(),
                                                                            rhand.fingers[f].joints[Leap::Bone::TYPE_METACARPAL].pos,
                                                                            LEAP_FILTER_COEFF, hand.confidence());
                    rhand.fingers[f].joints[Leap::Bone::TYPE_METACARPAL].width = mcp.width();
                    rhand.fingers[f].joints[Leap::Bone::TYPE_METACARPAL].length = mcp.length();
                    rhand.fingers[f].joints[Leap::Bone::TYPE_METACARPAL].orientation = LeapBasis_to_ktQuaternion(mcp.basis());
                    
                    // TODO: thumb?
                    for (int b = 0; b < 4; ++b) {
                        Leap::Bone bone = finger.bone(static_cast<Leap::Bone::Type>(b));
                        rhand.fingers[f].joints[b+1].pos = flt(bone.center(),
                                                        rhand.fingers[f].joints[b+1].pos,
                                                        LEAP_FILTER_COEFF, hand.confidence());
                        rhand.fingers[f].joints[b+1].width = mcp.width();
                        rhand.fingers[f].joints[b+1].length = mcp.length();
                        rhand.fingers[f].joints[b+1].orientation = LeapBasis_to_ktQuaternion(mcp.basis());
                    }
                }
            }
            
            // saw it, inc count
            saw = true;
            rfound = ::min(rfound+1,disappear_count);
        }
    }
    
    if (! saw)
    {
        int rf = rfound;
        rfound = max(rfound-1,0);
        
        if ((rf>0) && (rfound==0))
        {
            dbprintf(" --- (RIGHT) GONE (    )\n");
            
            rhand.is_valid = false;
            
            rhand.palm_location.pos = LEAP_JOINT_UNDEF;
            
            // hand is gone, default pos
            for (int f = 0; f < MAX_FINGERS; ++f) {
                
                // first joint inside hand, wrist joint
                rhand.fingers[f].joints[Leap::Bone::TYPE_METACARPAL].pos = LEAP_JOINT_UNDEF;
                for (int b = 0; b < MAX_BONES; ++b) {
                    
                    // // is_this_special_thumb_joint(f,b+1)
                    if ((f==0) && (b==0))
                    {
                        // this is "special" thumb extra bone w/ zero length, make it same as wrist joint
                        rhand.fingers[f].joints[b+1].pos = rhand.fingers[f].joints[Leap::Bone::TYPE_METACARPAL].pos;
                    }
                    else
                    {
                        rhand.fingers[f].joints[b+1].pos = LEAP_JOINT_UNDEF;
                    }
                }
            }
        }
    }
    
    return rhand;
}

//--------------------------
HandVector::hand_vecs leapDriver::get_left_hand_filtered(const Leap::Controller& controller) {
    
    bool saw = false;
    Leap::Frame frame = controller.frame();
    for (int h = 0; h < frame.hands().count(); ++h) {
        
        Leap::Hand hand = frame.hands()[h];
        if (hand.isLeft())
        {
            if ( ! lfound)
            {
                lhand = deep_copy_hand(hand,true);
                lhand.is_valid = true;
            }
            else
            {
                lhand.palm_location.pos = hand.stabilizedPalmPosition();
                
                for (int f = 0; f < hand.fingers().count(); ++f) {
                    Leap::Finger finger = hand.fingers()[f];
                    
                    // first joint inside hand.
                    Leap::Bone mcp = finger.bone(Leap::Bone::TYPE_METACARPAL);
                    lhand.fingers[f].joints[Leap::Bone::TYPE_METACARPAL].pos = flt(mcp.center(),
                                                                               lhand.fingers[f].joints[Leap::Bone::TYPE_METACARPAL].pos,
                                                                               LEAP_FILTER_COEFF, hand.confidence());
                    lhand.fingers[f].joints[Leap::Bone::TYPE_METACARPAL].width = mcp.width();
                    lhand.fingers[f].joints[Leap::Bone::TYPE_METACARPAL].length = mcp.length();
                    lhand.fingers[f].joints[Leap::Bone::TYPE_METACARPAL].orientation = LeapBasis_to_ktQuaternion(mcp.basis());
                    
                    // TODO: thumb ?
                    for (int b = 0; b < 4; ++b) {
                        Leap::Bone bone = finger.bone(static_cast<Leap::Bone::Type>(b));
                        lhand.fingers[f].joints[b+1].pos = flt(bone.center(),
                                                           lhand.fingers[f].joints[b+1].pos,
                                                           LEAP_FILTER_COEFF, hand.confidence());
                        lhand.fingers[f].joints[b+1].width = mcp.width();
                        lhand.fingers[f].joints[b+1].length = mcp.length();
                        lhand.fingers[f].joints[b+1].orientation = LeapBasis_to_ktQuaternion(mcp.basis());
                    }
                }
            }
            
            // saw it, inc count
            saw = true;
            lfound = ::min(lfound+1,disappear_count);
        }
    }
    
    if (! saw)
    {
        int rl = lfound;
        lfound = max(lfound-1,0);
        
        if ((rl>0) && (lfound==0))
        {
            dbprintf(" --- (     ) GONE (LEFT)\n");
            
            lhand.is_valid = false;
            
            lhand.palm_location.pos = LEAP_JOINT_UNDEF;
            
            // hand is gone, default pos
            for (int f = 0; f < MAX_FINGERS; ++f) {
                
                // first joint inside hand, wrist joint
                lhand.fingers[f].joints[Leap::Bone::TYPE_METACARPAL].pos = LEAP_JOINT_UNDEF;
                for (int b = 0; b < MAX_BONES; ++b) {
                    
                    // // is_this_special_thumb_joint(f,b+1)
                    if ((f==0) && (b==0))
                    {
                        // this is "special" thumb extra bone w/ zero length, make it same as wrist joint
                        lhand.fingers[f].joints[b+1].pos = lhand.fingers[f].joints[Leap::Bone::TYPE_METACARPAL].pos;
                    }
                    else
                    {
                        lhand.fingers[f].joints[b+1].pos = LEAP_JOINT_UNDEF;
                    }
                }
            }
        }
    }
    
    return lhand;
}


