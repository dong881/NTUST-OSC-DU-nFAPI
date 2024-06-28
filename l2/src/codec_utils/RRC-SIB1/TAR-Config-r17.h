/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_TAR_Config_r17_H_
#define	_TAR_Config_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum TAR_Config_r17__offsetThresholdTA_r17 {
	TAR_Config_r17__offsetThresholdTA_r17_ms0dot5	= 0,
	TAR_Config_r17__offsetThresholdTA_r17_ms1	= 1,
	TAR_Config_r17__offsetThresholdTA_r17_ms2	= 2,
	TAR_Config_r17__offsetThresholdTA_r17_ms3	= 3,
	TAR_Config_r17__offsetThresholdTA_r17_ms4	= 4,
	TAR_Config_r17__offsetThresholdTA_r17_ms5	= 5,
	TAR_Config_r17__offsetThresholdTA_r17_ms6	= 6,
	TAR_Config_r17__offsetThresholdTA_r17_ms7	= 7,
	TAR_Config_r17__offsetThresholdTA_r17_ms8	= 8,
	TAR_Config_r17__offsetThresholdTA_r17_ms9	= 9,
	TAR_Config_r17__offsetThresholdTA_r17_ms10	= 10,
	TAR_Config_r17__offsetThresholdTA_r17_ms11	= 11,
	TAR_Config_r17__offsetThresholdTA_r17_ms12	= 12,
	TAR_Config_r17__offsetThresholdTA_r17_ms13	= 13,
	TAR_Config_r17__offsetThresholdTA_r17_ms14	= 14,
	TAR_Config_r17__offsetThresholdTA_r17_ms15	= 15,
	TAR_Config_r17__offsetThresholdTA_r17_spare13	= 16,
	TAR_Config_r17__offsetThresholdTA_r17_spare12	= 17,
	TAR_Config_r17__offsetThresholdTA_r17_spare11	= 18,
	TAR_Config_r17__offsetThresholdTA_r17_spare10	= 19,
	TAR_Config_r17__offsetThresholdTA_r17_spare9	= 20,
	TAR_Config_r17__offsetThresholdTA_r17_spare8	= 21,
	TAR_Config_r17__offsetThresholdTA_r17_spare7	= 22,
	TAR_Config_r17__offsetThresholdTA_r17_spare6	= 23,
	TAR_Config_r17__offsetThresholdTA_r17_spare5	= 24,
	TAR_Config_r17__offsetThresholdTA_r17_spare4	= 25,
	TAR_Config_r17__offsetThresholdTA_r17_spare3	= 26,
	TAR_Config_r17__offsetThresholdTA_r17_spare2	= 27,
	TAR_Config_r17__offsetThresholdTA_r17_spare1	= 28
} e_TAR_Config_r17__offsetThresholdTA_r17;
typedef enum TAR_Config_r17__timingAdvanceSR_r17 {
	TAR_Config_r17__timingAdvanceSR_r17_enabled	= 0
} e_TAR_Config_r17__timingAdvanceSR_r17;

/* TAR-Config-r17 */
typedef struct TAR_Config_r17 {
	long	*offsetThresholdTA_r17;	/* OPTIONAL */
	long	*timingAdvanceSR_r17;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} TAR_Config_r17_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_offsetThresholdTA_r17_2;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_timingAdvanceSR_r17_32;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_TAR_Config_r17;
extern asn_SEQUENCE_specifics_t asn_SPC_TAR_Config_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_TAR_Config_r17_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _TAR_Config_r17_H_ */
#include <asn_internal.h>
