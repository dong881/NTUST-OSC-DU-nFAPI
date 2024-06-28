/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_MRDC_Parameters_v1630_H_
#define	_MRDC_Parameters_v1630_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum MRDC_Parameters_v1630__maxUplinkDutyCycle_interBandENDC_FDD_TDD_PC2_r16__maxUplinkDutyCycle_FDD_TDD_EN_DC1_r16 {
	MRDC_Parameters_v1630__maxUplinkDutyCycle_interBandENDC_FDD_TDD_PC2_r16__maxUplinkDutyCycle_FDD_TDD_EN_DC1_r16_n30	= 0,
	MRDC_Parameters_v1630__maxUplinkDutyCycle_interBandENDC_FDD_TDD_PC2_r16__maxUplinkDutyCycle_FDD_TDD_EN_DC1_r16_n40	= 1,
	MRDC_Parameters_v1630__maxUplinkDutyCycle_interBandENDC_FDD_TDD_PC2_r16__maxUplinkDutyCycle_FDD_TDD_EN_DC1_r16_n50	= 2,
	MRDC_Parameters_v1630__maxUplinkDutyCycle_interBandENDC_FDD_TDD_PC2_r16__maxUplinkDutyCycle_FDD_TDD_EN_DC1_r16_n60	= 3,
	MRDC_Parameters_v1630__maxUplinkDutyCycle_interBandENDC_FDD_TDD_PC2_r16__maxUplinkDutyCycle_FDD_TDD_EN_DC1_r16_n70	= 4,
	MRDC_Parameters_v1630__maxUplinkDutyCycle_interBandENDC_FDD_TDD_PC2_r16__maxUplinkDutyCycle_FDD_TDD_EN_DC1_r16_n80	= 5,
	MRDC_Parameters_v1630__maxUplinkDutyCycle_interBandENDC_FDD_TDD_PC2_r16__maxUplinkDutyCycle_FDD_TDD_EN_DC1_r16_n90	= 6,
	MRDC_Parameters_v1630__maxUplinkDutyCycle_interBandENDC_FDD_TDD_PC2_r16__maxUplinkDutyCycle_FDD_TDD_EN_DC1_r16_n100	= 7
} e_MRDC_Parameters_v1630__maxUplinkDutyCycle_interBandENDC_FDD_TDD_PC2_r16__maxUplinkDutyCycle_FDD_TDD_EN_DC1_r16;
typedef enum MRDC_Parameters_v1630__maxUplinkDutyCycle_interBandENDC_FDD_TDD_PC2_r16__maxUplinkDutyCycle_FDD_TDD_EN_DC2_r16 {
	MRDC_Parameters_v1630__maxUplinkDutyCycle_interBandENDC_FDD_TDD_PC2_r16__maxUplinkDutyCycle_FDD_TDD_EN_DC2_r16_n30	= 0,
	MRDC_Parameters_v1630__maxUplinkDutyCycle_interBandENDC_FDD_TDD_PC2_r16__maxUplinkDutyCycle_FDD_TDD_EN_DC2_r16_n40	= 1,
	MRDC_Parameters_v1630__maxUplinkDutyCycle_interBandENDC_FDD_TDD_PC2_r16__maxUplinkDutyCycle_FDD_TDD_EN_DC2_r16_n50	= 2,
	MRDC_Parameters_v1630__maxUplinkDutyCycle_interBandENDC_FDD_TDD_PC2_r16__maxUplinkDutyCycle_FDD_TDD_EN_DC2_r16_n60	= 3,
	MRDC_Parameters_v1630__maxUplinkDutyCycle_interBandENDC_FDD_TDD_PC2_r16__maxUplinkDutyCycle_FDD_TDD_EN_DC2_r16_n70	= 4,
	MRDC_Parameters_v1630__maxUplinkDutyCycle_interBandENDC_FDD_TDD_PC2_r16__maxUplinkDutyCycle_FDD_TDD_EN_DC2_r16_n80	= 5,
	MRDC_Parameters_v1630__maxUplinkDutyCycle_interBandENDC_FDD_TDD_PC2_r16__maxUplinkDutyCycle_FDD_TDD_EN_DC2_r16_n90	= 6,
	MRDC_Parameters_v1630__maxUplinkDutyCycle_interBandENDC_FDD_TDD_PC2_r16__maxUplinkDutyCycle_FDD_TDD_EN_DC2_r16_n100	= 7
} e_MRDC_Parameters_v1630__maxUplinkDutyCycle_interBandENDC_FDD_TDD_PC2_r16__maxUplinkDutyCycle_FDD_TDD_EN_DC2_r16;
typedef enum MRDC_Parameters_v1630__interBandMRDC_WithOverlapDL_Bands_r16 {
	MRDC_Parameters_v1630__interBandMRDC_WithOverlapDL_Bands_r16_supported	= 0
} e_MRDC_Parameters_v1630__interBandMRDC_WithOverlapDL_Bands_r16;

/* MRDC-Parameters-v1630 */
typedef struct MRDC_Parameters_v1630 {
	struct MRDC_Parameters_v1630__maxUplinkDutyCycle_interBandENDC_FDD_TDD_PC2_r16 {
		long	*maxUplinkDutyCycle_FDD_TDD_EN_DC1_r16;	/* OPTIONAL */
		long	*maxUplinkDutyCycle_FDD_TDD_EN_DC2_r16;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *maxUplinkDutyCycle_interBandENDC_FDD_TDD_PC2_r16;
	long	*interBandMRDC_WithOverlapDL_Bands_r16;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MRDC_Parameters_v1630_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_maxUplinkDutyCycle_FDD_TDD_EN_DC1_r16_3;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_maxUplinkDutyCycle_FDD_TDD_EN_DC2_r16_12;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_interBandMRDC_WithOverlapDL_Bands_r16_21;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_MRDC_Parameters_v1630;
extern asn_SEQUENCE_specifics_t asn_SPC_MRDC_Parameters_v1630_specs_1;
extern asn_TYPE_member_t asn_MBR_MRDC_Parameters_v1630_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _MRDC_Parameters_v1630_H_ */
#include <asn_internal.h>
