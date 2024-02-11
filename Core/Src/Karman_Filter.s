
.global updateKarmanFilter_ASM

updateKarmanFilter_ASM:

//load struct
VLDR.f32 S1, [R0, #0] //q
VLDR.f32 S2, [R0, #4] //p
VLDR.f32 S3, [R0, #8] //r
VLDR.f32 S4, [R0, #12] //k
VLDR.f32 S5, [R0, #16] //x

//update
VADD.f32 S2, S2, S1 //p = p+q
BVS Error
VADD.f32 S6, S2, S3 //temp = p+r
VMRS APSR_nzcv, FPSCR
BVS Error
VCMP.f32 S6, #0
VMRS APSR_nzcv, FPSCR
BEQ Error
VDIV.f32 S4, S2, S6 //k = p/(p+r)
VSUB.f32 S0, S0, S5 //measurement-x
VMRS APSR_nzcv, FPSCR
BVS Error
VMUL.f32 S6, S4, S0 //temp = k(m-x)
VMRS APSR_nzcv, FPSCR
BVS Error
VADD.f32 S5, S5, S6 //x = x+temp
VMRS APSR_nzcv, FPSCR
BVS Error
VMOV S6, #1
VSUB.f32 S6, S6, S4 //temp = 1-k//no err possible
VMUL.f32 S2, S2, S6 //p = p(1-k)
VMRS APSR_nzcv, FPSCR
BVS Error //overflow


//store
VSTR.f32 S1, [R0, #0]
VSTR.f32 S2, [R0, #4]
VSTR.f32 S3, [R0, #8]
VSTR.f32 S4, [R0, #12]
VSTR.f32 S5, [R0, #16]

MOV R0, #0 //return 0 if no err
MOV PC, LR

Error:
MOV R0, #1 //return -1 if error
MOV PC, LR

.end
