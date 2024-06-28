/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_MinTimeGap_r16_H_
#define	_MinTimeGap_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum MinTimeGap_r16__scs_15kHz_r16 {
	MinTimeGap_r16__scs_15kHz_r16_sl1	= 0,
	MinTimeGap_r16__scs_15kHz_r16_sl3	= 1
} e_MinTimeGap_r16__scs_15kHz_r16;
typedef enum MinTimeGap_r16__scs_30kHz_r16 {
	MinTimeGap_r16__scs_30kHz_r16_sl1	= 0,
	MinTimeGap_r16__scs_30kHz_r16_sl6	= 1
} e_MinTimeGap_r16__scs_30kHz_r16;
typedef enum MinTimeGap_r16__scs_60kHz_r16 {
	MinTimeGap_r16__scs_60kHz_r16_sl1	= 0,
	MinTimeGap_r16__scs_60kHz_r16_sl12	= 1
} e_MinTimeGap_r16__scs_60kHz_r16;
typedef enum MinTimeGap_r16__scs_120kHz_r16 {
	MinTimeGap_r16__scs_120kHz_r16_sl2	= 0,
	MinTimeGap_r16__scs_120kHz_r16_sl24	= 1
} e_MinTimeGap_r16__scs_120kHz_r16;

/* MinTimeGap-r16 */
typedef struct MinTimeGap_r16 {
	long	*scs_15kHz_r16;	/* OPTIONAL */
	long	*scs_30kHz_r16;	/* OPTIONAL */
	long	*scs_60kHz_r16;	/* OPTIONAL */
	long	*scs_120kHz_r16;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MinTimeGap_r16_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_scs_15kHz_r16_2;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_scs_30kHz_r16_5;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_scs_60kHz_r16_8;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_scs_120kHz_r16_11;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_MinTimeGap_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_MinTimeGap_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_MinTimeGap_r16_1[4];

#ifdef __cplusplus
}
#endif

#endif	/* _MinTimeGap_r16_H_ */
#include <asn_internal.h>
