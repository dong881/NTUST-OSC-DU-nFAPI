/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_CG_SDT_TA_ValidationConfig_r17_H_
#define	_CG_SDT_TA_ValidationConfig_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum CG_SDT_TA_ValidationConfig_r17__cg_SDT_RSRP_ChangeThreshold_r17 {
	CG_SDT_TA_ValidationConfig_r17__cg_SDT_RSRP_ChangeThreshold_r17_dB2	= 0,
	CG_SDT_TA_ValidationConfig_r17__cg_SDT_RSRP_ChangeThreshold_r17_dB4	= 1,
	CG_SDT_TA_ValidationConfig_r17__cg_SDT_RSRP_ChangeThreshold_r17_dB6	= 2,
	CG_SDT_TA_ValidationConfig_r17__cg_SDT_RSRP_ChangeThreshold_r17_dB8	= 3,
	CG_SDT_TA_ValidationConfig_r17__cg_SDT_RSRP_ChangeThreshold_r17_dB10	= 4,
	CG_SDT_TA_ValidationConfig_r17__cg_SDT_RSRP_ChangeThreshold_r17_dB14	= 5,
	CG_SDT_TA_ValidationConfig_r17__cg_SDT_RSRP_ChangeThreshold_r17_dB18	= 6,
	CG_SDT_TA_ValidationConfig_r17__cg_SDT_RSRP_ChangeThreshold_r17_dB22	= 7,
	CG_SDT_TA_ValidationConfig_r17__cg_SDT_RSRP_ChangeThreshold_r17_dB26	= 8,
	CG_SDT_TA_ValidationConfig_r17__cg_SDT_RSRP_ChangeThreshold_r17_dB30	= 9,
	CG_SDT_TA_ValidationConfig_r17__cg_SDT_RSRP_ChangeThreshold_r17_dB34	= 10,
	CG_SDT_TA_ValidationConfig_r17__cg_SDT_RSRP_ChangeThreshold_r17_spare5	= 11,
	CG_SDT_TA_ValidationConfig_r17__cg_SDT_RSRP_ChangeThreshold_r17_spare4	= 12,
	CG_SDT_TA_ValidationConfig_r17__cg_SDT_RSRP_ChangeThreshold_r17_spare3	= 13,
	CG_SDT_TA_ValidationConfig_r17__cg_SDT_RSRP_ChangeThreshold_r17_spare2	= 14,
	CG_SDT_TA_ValidationConfig_r17__cg_SDT_RSRP_ChangeThreshold_r17_spare1	= 15
} e_CG_SDT_TA_ValidationConfig_r17__cg_SDT_RSRP_ChangeThreshold_r17;

/* CG-SDT-TA-ValidationConfig-r17 */
typedef struct CG_SDT_TA_ValidationConfig_r17 {
	long	 cg_SDT_RSRP_ChangeThreshold_r17;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} CG_SDT_TA_ValidationConfig_r17_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_cg_SDT_RSRP_ChangeThreshold_r17_2;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_CG_SDT_TA_ValidationConfig_r17;
extern asn_SEQUENCE_specifics_t asn_SPC_CG_SDT_TA_ValidationConfig_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_CG_SDT_TA_ValidationConfig_r17_1[1];

#ifdef __cplusplus
}
#endif

#endif	/* _CG_SDT_TA_ValidationConfig_r17_H_ */
#include <asn_internal.h>
