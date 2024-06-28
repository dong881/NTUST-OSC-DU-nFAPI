/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_MPE_Config_FR2_r16_H_
#define	_MPE_Config_FR2_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum MPE_Config_FR2_r16__mpe_ProhibitTimer_r16 {
	MPE_Config_FR2_r16__mpe_ProhibitTimer_r16_sf0	= 0,
	MPE_Config_FR2_r16__mpe_ProhibitTimer_r16_sf10	= 1,
	MPE_Config_FR2_r16__mpe_ProhibitTimer_r16_sf20	= 2,
	MPE_Config_FR2_r16__mpe_ProhibitTimer_r16_sf50	= 3,
	MPE_Config_FR2_r16__mpe_ProhibitTimer_r16_sf100	= 4,
	MPE_Config_FR2_r16__mpe_ProhibitTimer_r16_sf200	= 5,
	MPE_Config_FR2_r16__mpe_ProhibitTimer_r16_sf500	= 6,
	MPE_Config_FR2_r16__mpe_ProhibitTimer_r16_sf1000	= 7
} e_MPE_Config_FR2_r16__mpe_ProhibitTimer_r16;
typedef enum MPE_Config_FR2_r16__mpe_Threshold_r16 {
	MPE_Config_FR2_r16__mpe_Threshold_r16_dB3	= 0,
	MPE_Config_FR2_r16__mpe_Threshold_r16_dB6	= 1,
	MPE_Config_FR2_r16__mpe_Threshold_r16_dB9	= 2,
	MPE_Config_FR2_r16__mpe_Threshold_r16_dB12	= 3
} e_MPE_Config_FR2_r16__mpe_Threshold_r16;

/* MPE-Config-FR2-r16 */
typedef struct MPE_Config_FR2_r16 {
	long	 mpe_ProhibitTimer_r16;
	long	 mpe_Threshold_r16;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MPE_Config_FR2_r16_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_mpe_ProhibitTimer_r16_2;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_mpe_Threshold_r16_11;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_MPE_Config_FR2_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_MPE_Config_FR2_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_MPE_Config_FR2_r16_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _MPE_Config_FR2_r16_H_ */
#include <asn_internal.h>
